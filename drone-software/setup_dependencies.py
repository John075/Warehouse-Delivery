import subprocess
import sys

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

# Install any packages not natively supported in rosdep
install("sseclient-py")
