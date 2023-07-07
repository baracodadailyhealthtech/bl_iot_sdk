How to get source voice raw data?

Step1: Enable Macro BLE_RC_VOICE_RAW_DATA_PRINT in customer_app\bl702l_remote_control\bl702l_remote_control\ble_rc_voice.c.
       And then compile bl702l_remote_control demo. Download the generated bin file to BL702L borad. Start up the borad.    

Step2: Source voice raw data will be printed out frame by frame via uart once voice key is pressed. Before you press voice key,
       Please use automatic save function of the serial tool to save log to a file, such as log.DAT which includes source voice data and other log.
	   Do not manually copy voice data, which will result in data loss. 
	   Once voice key is released, the voice data collection ends.

Step3: Use the python script customer_app\bl702l_remote_control\audio_parser.py to extract the source voice data to a file, such as source_voice_data.DAT.
       Below is the command.
	   python audio_parser.py log.DAT source_voice_data.DAT

Step4: Use your preferred PC tool to to open source_voice.DAT and play the voice.
       Take Cooledit tool for example,
	   #1.Set Sample Rate to 16000
	   #2.Set channels to Mono
	   #3.Set Resolution to 16-bit
	   #4.Set Data Formatted As 16-bit Intel PCM(LSB,MSB) in the menu list.