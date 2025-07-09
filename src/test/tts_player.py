#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TTS Player - Python版本
支持文本转语音，可配置发音人、语速、音量等参数
支持延时播放和音频文件保存
"""

import os
import sys
import time
import wave
import threading
from typing import Optional, List, Dict, Any
from dataclasses import dataclass
from pathlib import Path

try:
    import pyttsx3
    HAS_PYTTSX3 = True
except ImportError:
    HAS_PYTTSX3 = False
    print("Warning: pyttsx3 not installed. Install with: pip install pyttsx3")

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False
    print("Warning: pygame not installed. Install with: pip install pygame")

try:
    from gtts import gTTS
    import io
    HAS_GTTS = True
except ImportError:
    HAS_GTTS = False
    print("Warning: gtts not installed. Install with: pip install gtts")

@dataclass
class TTSConfig:
    """TTS配置类"""
    voice_name: str = "default"          # 发音人名称
    speed: int = 50                      # 语速 (0-100)
    volume: float = 0.8                  # 音量 (0.0-1.0)
    pitch: int = 50                      # 音调 (0-100)
    delay_seconds: float = 0.0           # 延时播放时间（秒）
    save_to_file: bool = True            # 是否保存音频文件
    output_file: str = "output.wav"      # 输出文件名
    play_audio: bool = True              # 是否播放音频
    language: str = "zh"                 # 语言代码
    engine_type: str = "pyttsx3"         # 引擎类型: pyttsx3, gtts
    output_dir: str = "audio_output"     # 输出目录

class TTSPlayer:
    """文本转语音播放器"""
    
    def __init__(self, config: Optional[TTSConfig] = None):
        self.config = config or TTSConfig()
        self.engine = None
        self.initialized = False
        self._setup_output_dir()
        
    def _setup_output_dir(self):
        """创建输出目录"""
        output_path = Path(self.config.output_dir)
        output_path.mkdir(exist_ok=True)
        
    def initialize(self) -> bool:
        """初始化TTS引擎"""
        if self.initialized:
            return True
            
        try:
            if self.config.engine_type == "pyttsx3" and HAS_PYTTSX3:
                self._init_pyttsx3()
            elif self.config.engine_type == "gtts" and HAS_GTTS:
                self._init_gtts()
            else:
                print(f"Engine {self.config.engine_type} not available, trying alternatives...")
                if HAS_PYTTSX3:
                    self.config.engine_type = "pyttsx3"
                    self._init_pyttsx3()
                elif HAS_GTTS:
                    self.config.engine_type = "gtts"
                    self._init_gtts()
                else:
                    raise Exception("No TTS engine available")
                    
            self.initialized = True
            print(f"TTS Player initialized with {self.config.engine_type} engine")
            return True
            
        except Exception as e:
            print(f"Failed to initialize TTS engine: {e}")
            return False
    
    def _init_pyttsx3(self):
        """初始化pyttsx3引擎"""
        self.engine = pyttsx3.init()
        
        # 设置语速
        rate = self.engine.getProperty('rate')
        new_rate = int(rate * (self.config.speed / 50.0))
        self.engine.setProperty('rate', new_rate)
        
        # 设置音量
        self.engine.setProperty('volume', self.config.volume)
        
        # 设置发音人
        voices = self.engine.getProperty('voices')
        if voices and self.config.voice_name != "default":
            for voice in voices:
                if self.config.voice_name.lower() in voice.name.lower():
                    self.engine.setProperty('voice', voice.id)
                    break
    
    def _init_gtts(self):
        """初始化gTTS引擎"""
        # gTTS不需要特殊初始化
        pass
    
    def synthesize_and_play(self, text: str) -> bool:
        """合成并播放语音"""
        if not self.initialized and not self.initialize():
            return False
        
        print(f"正在合成文本: \"{text}\"")
        print(f"发音人: {self.config.voice_name}, 语速: {self.config.speed}, 音量: {self.config.volume:.1f}")
        
        # 延时处理
        if self.config.delay_seconds > 0:
            print(f"等待 {self.config.delay_seconds} 秒后开始播放...")
            time.sleep(self.config.delay_seconds)
        
        try:
            if self.config.engine_type == "pyttsx3":
                return self._synthesize_pyttsx3(text)
            elif self.config.engine_type == "gtts":
                return self._synthesize_gtts(text)
            else:
                print(f"Unsupported engine: {self.config.engine_type}")
                return False
                
        except Exception as e:
            print(f"合成失败: {e}")
            return False
    
    def _synthesize_pyttsx3(self, text: str) -> bool:
        """使用pyttsx3合成语音"""
        output_path = None
        
        if self.config.save_to_file:
            output_path = Path(self.config.output_dir) / self.config.output_file
            self.engine.save_to_file(text, str(output_path))
        
        if self.config.play_audio:
            if output_path:
                # 先保存再播放
                self.engine.runAndWait()
                self._play_audio_file(str(output_path))
            else:
                # 直接播放
                self.engine.say(text)
                self.engine.runAndWait()
        elif self.config.save_to_file:
            # 仅保存
            self.engine.runAndWait()
        
        if output_path and output_path.exists():
            print(f"音频已保存到: {output_path}")
        
        return True
    
    def _synthesize_gtts(self, text: str) -> bool:
        """使用gTTS合成语音"""
        try:
            # 调整语速（gTTS通过slow参数控制）
            slow = self.config.speed < 50
            
            tts = gTTS(text=text, lang=self.config.language, slow=slow)
            
            if self.config.save_to_file:
                output_path = Path(self.config.output_dir) / self.config.output_file
                tts.save(str(output_path))
                print(f"音频已保存到: {output_path}")
                
                if self.config.play_audio:
                    self._play_audio_file(str(output_path))
            else:
                # 保存到临时文件并播放
                import tempfile
                with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp_file:
                    tts.save(tmp_file.name)
                    if self.config.play_audio:
                        self._play_audio_file(tmp_file.name)
                    os.unlink(tmp_file.name)
            
            return True
            
        except Exception as e:
            print(f"gTTS合成失败: {e}")
            return False
    
    def _play_audio_file(self, file_path: str):
        """播放音频文件"""
        if not HAS_PYGAME:
            print("pygame未安装，无法播放音频文件")
            return
        
        try:
            pygame.mixer.init()
            
            # 调整音量
            pygame.mixer.music.set_volume(self.config.volume)
            
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            
            # 等待播放完成
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)
                
        except Exception as e:
            print(f"播放音频失败: {e}")
        finally:
            pygame.mixer.quit()
    
    def synthesize_to_file(self, text: str, filename: str) -> bool:
        """仅合成到文件"""
        old_config = self.config
        self.config.play_audio = False
        self.config.save_to_file = True
        self.config.output_file = filename
        
        result = self.synthesize_and_play(text)
        self.config = old_config
        return result
    
    def play_audio_file(self, filename: str) -> bool:
        """播放音频文件"""
        file_path = Path(self.config.output_dir) / filename
        if not file_path.exists():
            print(f"文件不存在: {file_path}")
            return False
        
        self._play_audio_file(str(file_path))
        return True
    
    def set_voice(self, voice_name: str):
        """设置发音人"""
        self.config.voice_name = voice_name
        if self.initialized and self.config.engine_type == "pyttsx3":
            voices = self.engine.getProperty('voices')
            if voices:
                for voice in voices:
                    if voice_name.lower() in voice.name.lower():
                        self.engine.setProperty('voice', voice.id)
                        break
    
    def set_speed(self, speed: int):
        """设置语速 (0-100)"""
        self.config.speed = max(0, min(100, speed))
        if self.initialized and self.config.engine_type == "pyttsx3":
            rate = self.engine.getProperty('rate')
            new_rate = int(rate * (speed / 50.0))
            self.engine.setProperty('rate', new_rate)
    
    def set_volume(self, volume: float):
        """设置音量 (0.0-1.0)"""
        self.config.volume = max(0.0, min(1.0, volume))
        if self.initialized and self.config.engine_type == "pyttsx3":
            self.engine.setProperty('volume', volume)
    
    def set_delay(self, seconds: float):
        """设置延时播放时间（秒）"""
        self.config.delay_seconds = max(0.0, seconds)
    
    def get_available_voices(self) -> List[Dict[str, Any]]:
        """获取可用的发音人列表"""
        if not self.initialized:
            self.initialize()
        
        voices_info = []
        
        if self.config.engine_type == "pyttsx3" and self.engine:
            voices = self.engine.getProperty('voices')
            if voices:
                for voice in voices:
                    voices_info.append({
                        'id': voice.id,
                        'name': voice.name,
                        'language': getattr(voice, 'languages', ['unknown'])
                    })
        elif self.config.engine_type == "gtts":
            # gTTS支持的语言
            voices_info = [
                {'id': 'zh', 'name': '中文', 'language': ['zh']},
                {'id': 'en', 'name': 'English', 'language': ['en']},
                {'id': 'ja', 'name': '日本語', 'language': ['ja']},
                {'id': 'ko', 'name': '한국어', 'language': ['ko']}
            ]
        
        return voices_info
    
    def cleanup(self):
        """清理资源"""
        if self.engine and self.config.engine_type == "pyttsx3":
            try:
                self.engine.stop()
            except:
                pass
        self.initialized = False
    
    def __del__(self):
        self.cleanup()