@ECHO OFF
echo Grabbing and installing python first
powershell (New-Object Net.WebClient).DownloadFile('https://www.python.org/ftp/python/3.7.9/python-3.7.9-amd64.exe', '%~dp0python379_install.exe')
"%~dp0python379_install.exe" /passive Shortucts=0 Include_doc=0 Include_tcltk=1 Include_test=0 Include_tools=0 Include_launcher=0
"%AppData%\..\Local\Programs\Python\Python37\python.exe" -m pip install virtualenv
echo Done installing python
"%AppData%\..\Local\Programs\Python\Python37\python.exe" -m virtualenv eggbot_venv --clear
ecob Done setting up env
call eggbot_venv\Scripts\activate.bat
pip install matplotlib
pip install scipy
pip install PyQt6
pip install pyqtgraph
pip install opencv-python
pip install xmltodict
pip install pynput
pip install spyder
echo Done setup of dependancies and Spyder
echo @ECHO OFF > "%~dp0eggbot_venv\run_spyder.bat"
echo powershell -window hidden -command "" >> "%~dp0eggbot_venv\run_spyder.bat"
echo cd "%~dp0" >> %~dp0eggbot_venv\run_spyder.bat"
echo call "%~dp0eggbot_venv\Scripts\activate.bat" >> "%~dp0eggbot_venv\run_spyder.bat"
echo "%~dp0eggbot_venv\Scripts\spyder.exe" >> "%~dp0eggbot_venv\run_spyder.bat"
echo deactivate >> "%~dp0eggbot_venv\run_spyder.bat"
powershell "$s=(New-Object -COM WScript.Shell).CreateShortcut('%userprofile%\Start Menu\Programs\Spyder_eggbot_venv.lnk');$s.TargetPath='%~dp0eggbot_venv\run_spyder.bat';$s.IconLocation='%~dp0eggbot_venv\Lib\site-packages\spyder\images\windows_app_icon.ico';$s.Save()"
echo Created Spyder launcher and shortcut
del /Q "%~dp0python379_install.exe"
echo Done cleanup, finished!
pause