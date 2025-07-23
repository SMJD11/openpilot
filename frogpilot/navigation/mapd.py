#!/usr/bin/env python3
# PFEIFER - MAPD - Modified by FrogAi for FrogPilot
# CONFIGURED FOR CUSTOM MAPD WITH HARDCODED VERSION
import os
import shutil
import stat
import subprocess
import time
import urllib.request

from pathlib import Path

from openpilot.frogpilot.common.frogpilot_utilities import is_url_pingable
from openpilot.frogpilot.common.frogpilot_variables import MAPD_PATH, params_memory

# ==============================================================================
# ---                           CONFIGURATION                            ---
# ---    YOU ONLY NEED TO EDIT THE VARIABLES IN THIS SECTION             ---
# ==============================================================================

# 1. Your GitHub username.
GITHUB_USER = "SMJD11"

# 2. The name of your 'mapd' code repository.
MAPD_REPO = "mapd"

# 3. The EXACT tag of the GitHub Release you want to use.
#    This is the version of the mapd binary that will always be used.
#    Example: "v1.11.0"
MAPD_VERSION = "v1.2.1-stop-sign"

# ==============================================================================
# ---                         END OF CONFIGURATION                         ---
# ==============================================================================


# Local path to store the version info on the device. This lets the script
# know which version is currently installed.
VERSION_PATH = Path("/data/media/0/osm/mapd_version")

def download():
  """
  Downloads the specified mapd binary from YOUR GitHub releases.
  """
  Path(MAPD_PATH).parent.mkdir(parents=True, exist_ok=True)

  # Wait for an internet connection
  while not is_url_pingable("https://github.com"):
    time.sleep(60)

  # Construct the URL pointing to your specific mapd release asset.
  url = f"https://github.com/{GITHUB_USER}/{MAPD_REPO}/releases/download/{MAPD_VERSION}/mapd"

  try:
    print(f"Downloading your specified mapd version {MAPD_VERSION}...")
    with urllib.request.urlopen(url) as response:
      with open(MAPD_PATH, "wb") as mapd_file:
        shutil.copyfileobj(response, mapd_file)
        os.fsync(mapd_file.fileno())
        # Make the downloaded file executable
        os.chmod(MAPD_PATH, os.stat(MAPD_PATH).st_mode | stat.S_IEXEC)

    # Write the hardcoded version to the local version file to confirm it's installed.
    with open(VERSION_PATH, "w") as version_file:
      version_file.write(MAPD_VERSION)
      os.fsync(version_file.fileno())

    print(f"Successfully downloaded and installed mapd version {MAPD_VERSION}.")

  except Exception as error:
    print(f"Failed to download mapd from {url}: {error}")
    # If the download fails, delete the version file to force a retry on the next loop
    if VERSION_PATH.exists():
        VERSION_PATH.unlink()

def mapd_thread():
  """
  The main thread that manages the mapd lifecycle.
  """
  while True:
    try:
      # Check if the correct version is installed
      is_correct_version_installed = False
      if MAPD_PATH.exists() and VERSION_PATH.exists():
        with open(VERSION_PATH) as version_file:
          if version_file.read().strip() == MAPD_VERSION:
            is_correct_version_installed = True

      # If the correct version isn't installed, download it.
      if not is_correct_version_installed:
        print(f"Correct mapd version ({MAPD_VERSION}) not found. Downloading...")
        download()
        continue

      # Ensure the binary is executable
      if not os.access(MAPD_PATH, os.X_OK):
        print(f"{MAPD_PATH} is not executable. Fixing permissions...")
        os.chmod(MAPD_PATH, os.stat(MAPD_PATH).st_mode | stat.S_IEXEC)

      # Run the mapd binary
      process = subprocess.Popen(str(MAPD_PATH))
      process.wait() # Wait for the process to exit (e.g., if it crashes)

    except Exception as e:
      print(f"An error occurred in mapd_thread: {e}")
      # If something goes wrong, wait a bit before retrying
      time.sleep(30)

def main():
  params_memory.put("MapdLogLevel", "info") # Set a default log level
  mapd_thread()

if __name__ == "__main__":
  main()