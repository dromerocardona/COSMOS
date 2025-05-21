# GCS Cloning Instructions
## 1. Prepare an IDE
- E.g. Pycharm, VSCode, etc.
- Built in Git functionality/compatability is ideal
- Install Python 3.13 (or latest)
## 2. Clone the Repository
- Using Git, clone the contents of the repository
- Multiple methods of doing this
    - Use link: https://github.com/dromerocardona/COSMOS.git
    - Connect to GitHub account via your IDE and clone the repository
- Once done, all of the COSMOS files will be saved to your computer's disk in a folder
## 3. Installing Dependencies
- To run the GCS from your IDE, you will need a virtual environment (venv)
    - Moving to a venv varies between IDEs
- Ensure pip is installed in the venv (pip is a python package manager)
- Install the following libraries to the venv using pip (pip install [library]):
    - pyqt5
    - pyqtgraph
    - folium
    - offline_folium
    - pyserial
    - setuptools
    - playsound3
- Additional files are needed for the offline_folium library to function
    - These can be found in the folder entitled "offline_folium_requirements" in the GCS folder
    - Go to COSMOS/.venv/Lib/site_packages/offline_folium/local
    - Copy all the files from the "offline_folium_requirements" folder into the "local" folder
