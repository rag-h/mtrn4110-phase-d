import os
os.system('jupyter nbconvert --execute --clear-output PhaseC.ipynb')
print("Done Processing Images and Generating Map")
os.system('export WEBOTS_HOME=/Applications/Webots.app')
print("WEBOTS_HOME Defined")
os.system('cd /Applications/Webots.app')
os.system('ls')
os.system('/Applications/Webots.app/webots --mode=realtime PhaseAandB/worlds/PhaseD.wbt')
