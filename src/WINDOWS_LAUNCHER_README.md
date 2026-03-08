## ASV Windows Launcher

This document explains how to use the `windows_launcher.py` script to build a single
Windows `.exe` that encapsulates all the steps needed to run your ASV digital twin
on Windows using WSL, Ubuntu 24.04 and ROS2 Jazzy.

The goal is that **end users never need to type a command in a terminal** – they only
interact with the launcher GUI.

---

### 1. Overview

The launcher provides the following steps (mirroring your `Design.pdf`):

- **1–2. Environment check**
  - Check that **WSL** and **Ubuntu 24.04** are installed.
  - Optionally create an **Ubuntu user** (username + password) from the GUI.
  - Check that **ROS2 Jazzy** is installed inside Ubuntu (simple `ros2 --version` test).

- **3–4. Scripts and packages**
  - Clone or update the private GitHub repository `https://github.com/cjjcdd/ASV.git`
    into a folder named `ASV` next to the launcher exe.
  - Install or update Python dependencies for the project using
    `src/requirements.txt` inside Ubuntu.

- **5. Program update**
  - Check for and pull the latest version of the ASV repo (wrapper around `git fetch` / `git pull`).

- **6. Build**
  - Run
    `colcon build --packages-select asv_digital_twin asv_web_viewer`
    inside the WSL Ubuntu environment.

- **7. Run**
  - Start the **physics engine**:
    `ros2 launch asv_digital_twin master.launch.py`
  - Start the **dashboard GUI**:
    `python3 src/asv_digital_twin/asv_digital_twin/main_app.py`

All of these are run inside Ubuntu (WSL) via `wsl.exe`, but they are triggered
from buttons in the Windows launcher GUI.

---

### 2. Python dependencies (Ubuntu side)

Inside the repo, the file `src/requirements.txt` lists the Python packages used
by the ASV project:

- `PyQt5`
- `pyqtgraph`
- `numpy`
- `opencv-python`
- `pandas`
- `PyMuPDF`

These are installed inside **Ubuntu** (not Windows) by the launcher when you click:

- **Install Python Dependencies (requirements.txt) in Ubuntu**

The ROS2 dependencies (`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`,
`std_msgs`, `tf2_ros`, `ros_gz_bridge`, `ros_gz_sim`, `ros_gz_interfaces`,
`ros2launch`) come from the **ROS2 Jazzy** installation and are installed
via `apt` following the official ROS2 documentation.

When you add new Python packages in your nodes or GUI:

1. Add them to `src/requirements.txt`.
2. Commit and push your changes to GitHub.
3. End users can then:
   - Press **Check & Pull Latest ASV Version**.
   - Press **Install Python Dependencies** again.

---

### 3. Installing prerequisites (one-time)

#### 3.1 On Windows (developer machine)

You need Python 3 on Windows to build the `.exe` from `windows_launcher.py`.

1. Install **Python 3 (64-bit)** for Windows and ensure `python` and `pip`
   are on your PATH.

2. Install the Windows-side dependencies:

```bash
pip install PyQt5 pyinstaller
```

> Note: `PyInstaller` is used only on the developer machine to create the `.exe`.
> End users do not need Python installed on Windows.

3. Ensure **WSL** is enabled:

```powershell
wsl --install
```

4. Install **Ubuntu 24.04** as a WSL distribution:

```powershell
wsl --install -d Ubuntu-24.04
```

5. Open Ubuntu 24.04 at least once from the Start menu to complete its first-time setup.

6. Install **ROS2 Jazzy** inside Ubuntu 24.04 using the official documentation
   (this part is not automated by the launcher yet and should follow
   the instructions from the ROS2 docs).

---

### 4. Building the Windows `.exe` launcher

From the `src` folder (the same directory where `windows_launcher.py` lives), run:

```bash
pyinstaller --name ASVLauncher --onefile --noconsole windows_launcher.py
```

This creates:

- `dist/ASVLauncher.exe`

You can move `ASVLauncher.exe` to any folder where you want the ASV project
to live. The first time the user runs it, the launcher will:

- Clone the private ASV repo into a subfolder named `ASV` inside that folder.
- Use that path for all future builds and runs.

---

### 5. GitHub token handling

Because the repository `https://github.com/cjjcdd/ASV.git` is private, the launcher
needs a GitHub Personal Access Token (PAT) with `repo` access.

The launcher follows these **safe practices**:

- It never hard-codes your real token. Instead it uses a placeholder string
  `api` in the URL inside the code, which is replaced at runtime by whatever
  token the user provides.
- Tokens can be provided in two ways:

  1. **At runtime in the GUI**  
     The user pastes their token into the **GitHub Token** field on the launcher
     and clicks **Clone / Update Private ASV Repository**.

  2. **From a local environment variable**  
     The user creates a `.env` or otherwise sets an environment variable
     `GITHUB_TOKEN` on their machine, and leaves the GUI field empty.  
     The launcher will then read `GITHUB_TOKEN` and use it for cloning/pulling.

When constructing the URL, the launcher effectively does:

```text
https://<GITHUB_USERNAME>:<TOKEN>@github.com/cjjcdd/ASV.git
```

In the source code you will see:

```python
url = "https://cjjcdd:api@github.com/cjjcdd/ASV.git"
url = url.replace("api", token)
```

This keeps the real token out of the codebase.

---

### 6. First run on a new machine (end user)

Assuming you already built `ASVLauncher.exe` and distributed it:

1. **Place `ASVLauncher.exe` in an empty folder**  
   This folder will become the root for the ASV workspace (the repo will be cloned
   into a subfolder `ASV` here).

2. **Double-click `ASVLauncher.exe`**

3. **Step 1–2: Environment**

   - Optionally enter a desired Ubuntu username and password in the fields.
   - Click **Check / Prepare WSL + Ubuntu 24.04 + ROS2 Jazzy**.
   - The launcher will:
     - Verify that WSL is enabled and `Ubuntu-24.04` is installed.
     - Create the Ubuntu user if a username/password were provided.
     - Check for `ros2` inside Ubuntu and report if ROS2 Jazzy is installed.

4. **Step 3–4: Clone repo + Python deps**

   - On the next page, provide your **GitHub token** in the token field (or set
     `GITHUB_TOKEN` as an environment variable beforehand and leave the field empty).
   - Click **Clone / Update Private ASV Repository**:
     - On first run, this clones the repo into `./ASV` (relative to the exe).
     - On later runs, it just fetches and pulls new commits.
   - Click **Install Python Dependencies (requirements.txt) in Ubuntu**:
     - This runs `python3 -m pip install --user -r src/requirements.txt`
       inside Ubuntu.

5. **Step 5: Program update**

   - Click **Check & Pull Latest ASV Version** whenever you want to sync with
     the latest code on GitHub.

6. **Step 6–7: Build and run**

   - Click **Run colcon build (asv_digital_twin, asv_web_viewer)**:
     - This runs `colcon build --packages-select asv_digital_twin asv_web_viewer`
       inside Ubuntu using the cloned repo path.
   - After a successful build:
     - Click **Start Physics Engine (ROS2 launch)** to run  
       `ros2 launch asv_digital_twin master.launch.py`.
     - Click **Start Dashboard GUI** to run  
       `python3 src/asv_digital_twin/asv_digital_twin/main_app.py`.

The end user never needs to open a terminal: all steps are performed via GUI buttons.

---

### 7. Later runs / updating dependencies

On subsequent runs of the launcher:

- The repo already exists, so:
  - **Clone / Update** becomes effectively “update” (it runs `git fetch` / `git pull`).
  - **Install Python Dependencies** re-applies `requirements.txt` (safe if
    nothing changed, necessary if you added new packages).
  - **Run colcon build** only needs to be run when the code changed in a way
    that affects the binaries (but it is safe to re-run).

If you add new Python dependencies in the project:

1. Update `src/requirements.txt` with the new package names.
2. Commit and push the change to your private GitHub repo.
3. In the launcher:
   - Press **Check & Pull Latest ASV Version**.
   - Press **Install Python Dependencies** again.
   - Optionally **Run colcon build** again if necessary.

---

### 8. Notes and limitations

- The launcher assumes the WSL distribution is named exactly `Ubuntu-24.04`.
  If your system uses a different name, you can adjust `self.wsl_distro` in
  `windows_launcher.py`.
- ROS2 Jazzy itself must be installed inside Ubuntu 24.04 beforehand, following
  the official ROS2 documentation. The launcher only checks for `ros2` and logs
  the version – it does not yet run the full ROS2 installation script.
- All ROS-dependent binaries and Python modules (`rclpy`, `geometry_msgs`,
  `sensor_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`, `ros_gz_*`) are provided by
  the ROS2 installation and not by `pip`.

