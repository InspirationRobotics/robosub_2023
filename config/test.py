import os
from dotenv import load_dotenv

load_dotenv("onyx.env")

bottomMXID = os.getenv('bottomOak_camera_mxid')

print(bottomMXID)