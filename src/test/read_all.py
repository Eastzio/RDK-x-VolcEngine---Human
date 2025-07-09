import bme680
import time
import threading
from typing import Dict, Optional

class EnvironmentSensor:
    """环境传感器控制器 - 适配Function Calling"""
    
    def __init__(self):
        self.sensor = None
        self.is_initialized = False
        self.lock = threading.Lock()
        self.last_reading = None
        self.reading_thread = None
        self.continuous_reading = False
        
    def initialize(self) -> Dict[str, any]:
        """初始化环境传感器"""
        try:
            with self.lock:
                if self.is_initialized:
                    return {"success": True, "message": "传感器已初始化"}
                
                # 尝试连接传感器
                try:
                    self.sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
                except (RuntimeError, IOError):
                    self.sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)
                
                # 配置传感器
                self.sensor.set_humidity_oversample(bme680.OS_2X)
                self.sensor.set_pressure_oversample(bme680.OS_4X)
                self.sensor.set_temperature_oversample(bme680.OS_8X)
                self.sensor.set_filter(bme680.FILTER_SIZE_3)
                self.sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
                
                # 配置气体传感器
                self.sensor.set_gas_heater_temperature(320)
                self.sensor.set_gas_heater_duration(150)
                self.sensor.select_gas_heater_profile(0)
                
                self.is_initialized = True
                return {
                    "success": True,
                    "message": "环境传感器初始化成功",
                    "sensor_info": self._get_calibration_info()
                }
                
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _get_calibration_info(self) -> Dict[str, any]:
        """获取传感器校准信息"""
        calibration_data = {}
        for name in dir(self.sensor.calibration_data):
            if not name.startswith('_'):
                value = getattr(self.sensor.calibration_data, name)
                if isinstance(value, int):
                    calibration_data[name] = value
        return calibration_data
    
    def read_environment_data(self) -> Dict[str, any]:
        """读取环境数据"""
        if not self.is_initialized:
            init_result = self.initialize()
            if not init_result["success"]:
                return init_result
        
        try:
            with self.lock:
                if self.sensor.get_sensor_data():
                    data = {
                        "success": True,
                        "timestamp": time.time(),
                        "temperature": round(self.sensor.data.temperature, 2),  # 摄氏度
                        "pressure": round(self.sensor.data.pressure, 2),        # hPa
                        "humidity": round(self.sensor.data.humidity, 2),        # %RH
                        "gas_resistance": None,
                        "heat_stable": self.sensor.data.heat_stable
                    }
                    
                    # 如果气体传感器稳定，添加气体阻值
                    if self.sensor.data.heat_stable:
                        data["gas_resistance"] = self.sensor.data.gas_resistance  # Ohms
                    
                    self.last_reading = data
                    return data
                else:
                    return {"success": False, "error": "传感器数据读取失败"}
                    
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_air_quality_level(self) -> Dict[str, any]:
        """获取空气质量等级"""
        data = self.read_environment_data()
        if not data["success"]:
            return data
        
        # 基于气体阻值判断空气质量
        if data["gas_resistance"] is None:
            quality_level = "未知"
            quality_score = 0
        else:
            gas_resistance = data["gas_resistance"]
            if gas_resistance > 50000:
                quality_level = "优秀"
                quality_score = 5
            elif gas_resistance > 20000:
                quality_level = "良好"
                quality_score = 4
            elif gas_resistance > 10000:
                quality_level = "一般"
                quality_score = 3
            elif gas_resistance > 5000:
                quality_level = "较差"
                quality_score = 2
            else:
                quality_level = "差"
                quality_score = 1
        
        return {
            "success": True,
            "air_quality_level": quality_level,
            "air_quality_score": quality_score,
            "gas_resistance": data["gas_resistance"],
            "temperature": data["temperature"],
            "humidity": data["humidity"],
            "pressure": data["pressure"]
        }
    
    def get_comfort_index(self) -> Dict[str, any]:
        """获取舒适度指数"""
        data = self.read_environment_data()
        if not data["success"]:
            return data
        
        temp = data["temperature"]
        humidity = data["humidity"]
        
        # 计算舒适度指数
        comfort_score = 0
        comfort_factors = []
        
        # 温度舒适度 (18-26°C为最佳)
        if 18 <= temp <= 26:
            temp_score = 5
            comfort_factors.append("温度适宜")
        elif 15 <= temp < 18 or 26 < temp <= 30:
            temp_score = 3
            comfort_factors.append("温度偏离适宜范围")
        else:
            temp_score = 1
            comfort_factors.append("温度不适宜")
        
        # 湿度舒适度 (40-60%为最佳)
        if 40 <= humidity <= 60:
            humidity_score = 5
            comfort_factors.append("湿度适宜")
        elif 30 <= humidity < 40 or 60 < humidity <= 70:
            humidity_score = 3
            comfort_factors.append("湿度偏离适宜范围")
        else:
            humidity_score = 1
            comfort_factors.append("湿度不适宜")
        
        comfort_score = (temp_score + humidity_score) / 2
        
        if comfort_score >= 4.5:
            comfort_level = "非常舒适"
        elif comfort_score >= 3.5:
            comfort_level = "舒适"
        elif comfort_score >= 2.5:
            comfort_level = "一般"
        else:
            comfort_level = "不舒适"
        
        return {
            "success": True,
            "comfort_level": comfort_level,
            "comfort_score": round(comfort_score, 1),
            "comfort_factors": comfort_factors,
            "temperature": temp,
            "humidity": humidity
        }
    
    def start_continuous_monitoring(self, interval: float = 1.0) -> Dict[str, any]:
        """开始连续监测"""
        if self.continuous_reading:
            return {"success": False, "error": "连续监测已在运行"}
        
        def monitor_loop():
            while self.continuous_reading:
                self.read_environment_data()
                time.sleep(interval)
        
        self.continuous_reading = True
        self.reading_thread = threading.Thread(target=monitor_loop, daemon=True)
        self.reading_thread.start()
        
        return {"success": True, "message": f"开始连续监测，间隔{interval}秒"}
    
    def stop_continuous_monitoring(self) -> Dict[str, any]:
        """停止连续监测"""
        self.continuous_reading = False
        if self.reading_thread:
            self.reading_thread.join(timeout=2)
        return {"success": True, "message": "连续监测已停止"}
    
    def get_last_reading(self) -> Dict[str, any]:
        """获取最后一次读取的数据"""
        if self.last_reading:
            return self.last_reading
        else:
            return {"success": False, "error": "无可用数据"}

# Function Calling接口
environment_sensor = EnvironmentSensor()

def read_environment() -> Dict[str, any]:
    """读取环境温湿度及空气质量数据"""
    return environment_sensor.read_environment_data()

def get_air_quality() -> Dict[str, any]:
    """获取空气质量等级评估"""
    return environment_sensor.get_air_quality_level()

def get_comfort_level() -> Dict[str, any]:
    """获取环境舒适度评估"""
    return environment_sensor.get_comfort_index()

def start_environment_monitoring(interval: float = 1.0) -> Dict[str, any]:
    """开始连续环境监测
    Args:
        interval: 监测间隔时间(秒)
    """
    return environment_sensor.start_continuous_monitoring(interval)

def stop_environment_monitoring() -> Dict[str, any]:
    """停止连续环境监测"""
    return environment_sensor.stop_continuous_monitoring()

def get_last_environment_data() -> Dict[str, any]:
    """获取最后一次环境数据"""
    return environment_sensor.get_last_reading()

if __name__ == "__main__":
    # 测试代码
    print("环境传感器测试")
    result = read_environment()
    print(f"环境数据: {result}")
    
    quality = get_air_quality()
    print(f"空气质量: {quality}")
    
    comfort = get_comfort_level()
    print(f"舒适度: {comfort}")