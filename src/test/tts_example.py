#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TTS使用示例
"""

import time
from tts_player import TTSPlayer, TTSConfig

def main():
    print("=== Python TTS播放器示例 ===")
    
    # 创建TTS播放器实例
    config = TTSConfig(
        voice_name="default",
        speed=60,                    # 语速60%
        volume=0.8,                  # 音量80%
        delay_seconds=1.0,           # 延时1秒
        save_to_file=True,           # 保存音频文件
        output_file="hello.wav",     # 输出文件名
        play_audio=True,             # 同时播放音频
        engine_type="pyttsx3"        # 使用pyttsx3引擎
    )
    
    tts_player = TTSPlayer(config)
    
    # 初始化TTS引擎
    if not tts_player.initialize():
        print("TTS引擎初始化失败")
        return
    
    # 示例1：基本文本转语音
    print("\n=== 示例1：基本TTS ===")
    tts_player.synthesize_and_play("你好，这是一个Python版本的文本转语音测试。")
    
    # 示例2：调整参数
    print("\n=== 示例2：调整参数 ===")
    tts_player.set_speed(30)      # 较慢语速
    tts_player.set_volume(0.9)    # 较大音量
    tts_player.set_delay(0.5)     # 延时0.5秒
    tts_player.synthesize_and_play("现在语速比较慢，音量比较大。")
    
    # 示例3：仅保存到文件
    print("\n=== 示例3：仅保存文件 ===")
    tts_player.synthesize_to_file("这段语音只保存到文件，不播放。", "save_only.wav")
    
    # 示例4：播放已保存的音频文件
    print("\n=== 示例4：播放音频文件 ===")
    time.sleep(1)
    tts_player.play_audio_file("save_only.wav")
    
    # 示例5：显示可用的发音人
    print("\n=== 示例5：可用发音人 ===")
    voices = tts_player.get_available_voices()
    for i, voice in enumerate(voices[:5]):  # 只显示前5个
        print(f"{i+1}. {voice['name']} (ID: {voice['id']})")
    
    # 示例6：使用gTTS引擎
    print("\n=== 示例6：gTTS引擎 ===")
    gtts_config = TTSConfig(
        engine_type="gtts",
        language="zh",
        speed=40,
        output_file="gtts_output.mp3"
    )
    gtts_player = TTSPlayer(gtts_config)
    if gtts_player.initialize():
        gtts_player.synthesize_and_play("这是使用Google TTS引擎合成的语音。")
    
    print("\n=== 测试完成 ===")

if __name__ == "__main__":
    main()