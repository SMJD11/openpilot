# PFEIFER - MAPD - Modified by FrogAi for FrogPilot
#!/usr/bin/env python3
import os
import shutil
import stat
import subprocess
import time
import urllib.request

from pathlib import Path

from openpilot.selfdrive.frogpilot.frogpilot_utilities import is_url_pingable
from openpilot.selfdrive.frogpilot.frogpilot_variables import MAPD_PATH

VERSION = "stop-sign-alpha"  # Modified: Hardcoded to "stop-sign-alpha"

# Commented out original version URLS for direct download
# GITHUB_VERSION_URL = f"https://github.com/FrogAi/FrogPilot-Resources/raw/Versions/mapd_version_{VERSION}.json"
# GITLAB_VERSION_URL = f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Versions/mapd_version_{VERSION}.json"

VERSION_PATH = Path("/data/media/0/osm/mapd_version")

def download(): # Modified: Removed current_version parameter
  # Modified: Hardcoded download URL for stop-sign-alpha release
  urls = [
    f"https://github.com/SMJD11/mapd/releases/download/stop-sign-alpha/mapd", # Direct download URL - CHANGE THIS BACK TO ORIGINAL FOR VERSIONED RELEASES
    # f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Mapd/{current_version}" # Commented out original URL
  ]

  for url in urls:
    try:
      with urllib.request.urlopen(url, timeout=5) as response:
        with open(MAPD_PATH, 'wb') as mapd:
          shutil.copyfileobj(response, mapd)
          os.fsync(mapd.fileno())
          os.chmod(MAPD_PATH, os.stat(MAPD_PATH).st_mode | stat.S_IEXEC)
      # Commented out version writing for direct download
      # VERSION_PATH.write_text(current_version)
      # os.fsync(version_file.fileno())
      print(f"Successfully downloaded mapd from {url}")
      return True
    except Exception as error:
      print(f"Failed to download mapd from {url}: {error}")

  print(f"Failed to download mapd") # Modified: Removed version from error message
  return False

# Commented out original get_installed_version function for direct download
# def get_installed_version():
#   try:
#     return VERSION_PATH.read_text().strip()
#   except FileNotFoundError:
#     return None
#   except Exception as error:
#     print(f"Error reading installed version: {error}")
#     return None

# Commented out original get_latest_version function for direct download
# def get_latest_version():
#   for url in [GITHUB_VERSION_URL, GITLAB_VERSION_URL]:
#     try:
#       with urllib.request.urlopen(url, timeout=5) as response:
#         return json.loads(response.read().decode('utf-8'))['version']
#     except Exception as error:
#       print(f"Error fetching mapd version from {url}: {error}")
#   print("Failed to get the latest mapd version")
#   return None

# Commented out original update_mapd function for direct download
# def update_mapd():
#   installed_version = get_installed_version()
#   latest_version = get_latest_version()

#   if latest_version is None:
#     print("Could not get the latest mapd version")
#     return

#   if installed_version != latest_version:
#     print("New mapd version available, stopping the mapd process for update")
#     try:
#       subprocess.run(["pkill", "-f", MAPD_PATH], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#     except Exception as error:
#       print(f"Error stopping mapd process: {error}")

#     if download(latest_version):
#       print(f"Updated mapd to version {latest_version}")
#     else:
#       print("Failed to update mapd")
#   else:
#     print("Mapd is up to date")

def mapd_thread():
  if os.path.exists(MAPD_PATH) and os.path.isdir(MAPD_PATH):
    shutil.rmtree(MAPD_PATH)

  while True:
    if not os.path.exists(MAPD_PATH):
      print(f"{MAPD_PATH} not found. Downloading...")
      download() # Modified: Call download without arguments
      continue

    # Commented out version check for direct download
    # if not os.path.exists(VERSION_PATH):
    #   download()
    #   continue

    # with open(VERSION_PATH) as version_file:
    #   if is_url_pingable("https://github.com") or is_url_pingable("https://gitlab.com"):
    #     if version_file.read().strip() != get_latest_version():
    #       print("New mapd version available. Downloading...")
    #       download()
    #       continue

    process = subprocess.Popen(MAPD_PATH)
    process.wait()

def main():
  mapd_thread()

if __name__ == "__main__":
  main()