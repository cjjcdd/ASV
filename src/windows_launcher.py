import os
import sys
import subprocess
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QLineEdit,
    QStackedWidget,
    QTextEdit,
    QMessageBox,
)
from PyQt5.QtCore import Qt


def windows_to_wsl_path(win_path: str) -> str:
    """
    Convert an absolute Windows path (e.g. C:\\Users\\me\\ASV)
    into the equivalent WSL path (/mnt/c/Users/me/ASV).
    """
    win_path = os.path.abspath(win_path)
    drive, tail = os.path.splitdrive(win_path)
    if not drive:
        raise ValueError(f"Expected absolute Windows path, got: {win_path}")
    drive_letter = drive[0].lower()
    tail = tail.replace("\\", "/")
    return f"/mnt/{drive_letter}{tail}"


def run_cmd(cmd, *, cwd=None, capture_output=True, check=False):
    """
    Run a short Windows-side command without showing a terminal window.
    """
    result = subprocess.run(
        cmd,
        cwd=cwd,
        capture_output=capture_output,
        text=True,
        shell=False,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"Command failed: {' '.join(cmd)}\n"
            f"stdout:\n{result.stdout}\n\nstderr:\n{result.stderr}"
        )
    return result


def run_wsl_cmd(wsl_command: str, *, distro: str = "Ubuntu-24.04", check=False):
    """
    Run a bash command inside a WSL distro non-interactively.

    Example:
        run_wsl_cmd('cd /mnt/c/ASV && ls')
    """
    cmd = ["wsl", "-d", distro, "--", "bash", "-lc", wsl_command]
    result = run_cmd(cmd, capture_output=True, check=False)
    if check and result.returncode != 0:
        raise RuntimeError(
            f"WSL command failed: {wsl_command}\n"
            f"stdout:\n{result.stdout}\n\nstderr:\n{result.stderr}"
        )
    return result


class LauncherWindow(QMainWindow):
    """
    Windows launcher for the ASV digital twin.

    Flow (matches Design.pdf):
      1–2. Check Ubuntu 24.04 + ROS2 Jazzy, manage Ubuntu user.
      3–4. Clone / update private GitHub repo, install Python deps.
      5.   Check for program updates (git pull).
      6.   Build physics engine (colcon build).
      7.   Start physics engine and dashboard GUI.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ASV Windows Launcher")
        self.resize(1000, 650)

        # Determine "project root" as the folder containing the EXE (or this script).
        if getattr(sys, "frozen", False):
            self.project_root = Path(sys.executable).resolve().parent
        else:
            self.project_root = Path(__file__).resolve().parent

        # The private ASV repo will be cloned into a subfolder next to the launcher.
        # Example: <launcher_folder>/ASV
        self.repo_dir = self.project_root / "ASV"

        # WSL related
        self.wsl_distro = "Ubuntu-24.04"
        self.wsl_user = None  # optional, created via UI

        # GitHub related
        self.github_username = "cjjcdd"
        self.github_token = None  # user-provided or from local env

        # --- UI structure: main stack + log panel ---
        self.stack = QStackedWidget()
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.addWidget(self.stack)
        layout.addWidget(QLabel("Log:"))
        layout.addWidget(self.log_box)
        self.setCentralWidget(container)

        # Build pages
        self.page_env = self._build_env_page()
        self.page_repo = self._build_repo_page()
        self.page_update = self._build_update_page()
        self.page_run = self._build_run_page()

        self.stack.addWidget(self.page_env)
        self.stack.addWidget(self.page_repo)
        self.stack.addWidget(self.page_update)
        self.stack.addWidget(self.page_run)

    # ------------------------------------------------------------------
    # PAGE 1–2: Ubuntu 24.04 + ROS2 Jazzy + Ubuntu user
    # ------------------------------------------------------------------
    def _build_env_page(self) -> QWidget:
        page = QWidget()
        v = QVBoxLayout(page)

        title = QLabel("<h2>1–2. Ubuntu 24.04 + ROS2 Jazzy</h2>")
        title.setTextFormat(Qt.RichText)
        v.addWidget(title)

        desc = QLabel(
            "This step checks that Windows Subsystem for Linux (WSL) with "
            "Ubuntu 24.04 is installed, and that ROS2 Jazzy is available.\n\n"
            "You can also create a dedicated Ubuntu user for this simulation here."
        )
        desc.setWordWrap(True)
        v.addWidget(desc)

        self.txt_wsl_user = QLineEdit()
        self.txt_wsl_user.setPlaceholderText("Ubuntu username (optional)")
        self.txt_wsl_pass = QLineEdit()
        self.txt_wsl_pass.setPlaceholderText("Ubuntu password (optional)")
        self.txt_wsl_pass.setEchoMode(QLineEdit.Password)

        v.addWidget(QLabel("Ubuntu (WSL) user credentials:"))
        v.addWidget(self.txt_wsl_user)
        v.addWidget(self.txt_wsl_pass)

        btn_check = QPushButton("Check / Prepare WSL + Ubuntu 24.04 + ROS2 Jazzy")
        btn_check.clicked.connect(self.handle_check_env)
        v.addWidget(btn_check)

        nav_layout = QHBoxLayout()
        nav_layout.addStretch()
        btn_next = QPushButton("Next ▶")
        btn_next.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        nav_layout.addWidget(btn_next)
        v.addLayout(nav_layout)

        return page

    def handle_check_env(self):
        self.log("Checking WSL installation...")
        res = run_cmd(["wsl", "-l", "-v"], capture_output=True)
        if res.returncode != 0:
            QMessageBox.critical(
                self,
                "WSL not available",
                "WSL does not appear to be enabled on this system.\n\n"
                "Enable it from Windows Features or run:\n"
                "  wsl --install\n\n"
                "Then reboot and start this launcher again.",
            )
            self.log(res.stderr)
            return

        self.log(res.stdout)
        if self.wsl_distro not in res.stdout:
            QMessageBox.warning(
                self,
                "Ubuntu 24.04 missing",
                f"The '{self.wsl_distro}' distribution is not installed.\n\n"
                "Install Ubuntu 24.04 from Microsoft Store or run:\n"
                f"  wsl --install -d {self.wsl_distro}\n\n"
                "Then open Ubuntu once to finish initialization and re-run this app.",
            )
            return

        # Optionally create an Ubuntu user non-interactively.
        username = self.txt_wsl_user.text().strip()
        password = self.txt_wsl_pass.text().strip()
        if username and password:
            try:
                self.create_wsl_user(username, password)
                self.wsl_user = username
                QMessageBox.information(
                    self,
                    "WSL user",
                    f"Ubuntu user '{username}' is ready for use inside WSL.",
                )
            except Exception as e:
                QMessageBox.critical(self, "WSL user error", str(e))
                return

        # Basic ROS2 check: will simply log result (installation scripts are manual or separate).
        self.log("Checking ROS2 Jazzy in Ubuntu...")
        try:
            res_ros = run_wsl_cmd("which ros2 && ros2 --version", distro=self.wsl_distro)
            if res_ros.returncode == 0:
                self.log("ROS2 detected:\n" + res_ros.stdout)
                QMessageBox.information(
                    self,
                    "ROS2 detected",
                    "ROS2 is installed in Ubuntu. See log for version details.",
                )
            else:
                self.log(res_ros.stdout + "\n" + res_ros.stderr)
                QMessageBox.warning(
                    self,
                    "ROS2 not detected",
                    "ROS2 Jazzy was not found inside Ubuntu 24.04.\n"
                    "Please install ROS2 Jazzy following the official instructions, "
                    "then re-run this step.",
                )
        except Exception as e:
            self.log(f"ROS2 check failed: {e}")

    def create_wsl_user(self, username: str, password: str):
        """
        Ensure a user exists inside the Ubuntu distro with the specified password.
        """
        self.log(f"Ensuring Ubuntu user '{username}' exists...")
        cmd_check = f"id -u {username} >/dev/null 2>&1 || echo MISSING"
        res = run_wsl_cmd(cmd_check, distro=self.wsl_distro)
        if "MISSING" in res.stdout:
            self.log(f"Creating user {username}...")
            run_wsl_cmd(
                f"sudo useradd -m {username} && "
                f"echo '{username}:{password}' | sudo chpasswd && "
                f"sudo usermod -aG sudo {username}",
                distro=self.wsl_distro,
                check=True,
            )
        else:
            self.log(f"User '{username}' already exists in Ubuntu.")

    # ------------------------------------------------------------------
    # PAGE 3–4: GitHub repo clone / update + Python dependencies
    # ------------------------------------------------------------------
    def _build_repo_page(self) -> QWidget:
        page = QWidget()
        v = QVBoxLayout(page)

        title = QLabel("<h2>3–4. Get ASV Scripts & Python Packages</h2>")
        title.setTextFormat(Qt.RichText)
        v.addWidget(title)

        desc = QLabel(
            "This step clones or updates the private GitHub repository and "
            "installs Python dependencies from requirements.txt inside Ubuntu."
        )
        desc.setWordWrap(True)
        v.addWidget(desc)

        self.txt_token = QLineEdit()
        self.txt_token.setPlaceholderText(
            "GitHub token (or leave empty if using local environment variable GITHUB_TOKEN)"
        )
        self.txt_token.setEchoMode(QLineEdit.Password)

        v.addWidget(QLabel("GitHub Personal Access Token:"))
        v.addWidget(self.txt_token)

        btn_clone = QPushButton("Clone / Update Private ASV Repository")
        btn_clone.clicked.connect(self.handle_clone_repo)
        v.addWidget(btn_clone)

        btn_deps = QPushButton("Install Python Dependencies (requirements.txt) in Ubuntu")
        btn_deps.clicked.connect(self.handle_install_python_deps)
        v.addWidget(btn_deps)

        nav_layout = QHBoxLayout()
        nav_layout.addStretch()
        btn_next = QPushButton("Next ▶")
        btn_next.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        nav_layout.addWidget(btn_next)
        v.addLayout(nav_layout)

        return page

    def handle_clone_repo(self):
        """
        Clone the private GitHub repo into <launcher_folder>/ASV,
        or if it already exists, pull the latest changes.
        """
        token = self.txt_token.text().strip() or os.getenv("GITHUB_TOKEN", "")
        if not token:
            QMessageBox.warning(
                self,
                "Token missing",
                "Please provide a GitHub token here or set GITHUB_TOKEN in a local .env / environment.",
            )
            return
        self.github_token = token

        if not self.repo_dir.exists():
            self.log("Cloning ASV repository (first-time setup)...")
            # NOTE: 'api' is a placeholder; it is replaced immediately at runtime by the real token.
            url = "https://cjjcdd:api@github.com/cjjcdd/ASV.git"
            url = url.replace("api", token)

            wsl_project_root = windows_to_wsl_path(str(self.project_root))
            cmd = f"cd {wsl_project_root} && git clone {url} ASV"
            res = run_wsl_cmd(cmd, distro=self.wsl_distro)
            self.log(res.stdout + "\n" + res.stderr)
            if res.returncode != 0:
                QMessageBox.critical(self, "Clone failed", "Failed to clone repository. See log for details.")
            else:
                QMessageBox.information(self, "Clone complete", "Repository cloned successfully.")
        else:
            self.log("Repository already exists. Fetching and pulling latest changes...")
            wsl_repo = windows_to_wsl_path(str(self.repo_dir))
            cmd = f"cd {wsl_repo} && git fetch origin && git pull"
            res = run_wsl_cmd(cmd, distro=self.wsl_distro)
            self.log(res.stdout + "\n" + res.stderr)
            if res.returncode != 0:
                QMessageBox.critical(self, "Update failed", "Failed to update repository. See log for details.")
            else:
                QMessageBox.information(self, "Update complete", "Repository updated to latest version.")

    def handle_install_python_deps(self):
        """
        Install or update Python dependencies inside Ubuntu using requirements.txt.
        """
        if not self.repo_dir.exists():
            QMessageBox.warning(self, "Repository missing", "Clone the repository first.")
            return

        wsl_repo = windows_to_wsl_path(str(self.repo_dir))
        cmd = f"cd {wsl_repo} && python3 -m pip install --user -r src/requirements.txt"
        self.log("Installing Python dependencies inside Ubuntu...")
        res = run_wsl_cmd(cmd, distro=self.wsl_distro)
        self.log(res.stdout + "\n" + res.stderr)
        if res.returncode == 0:
            QMessageBox.information(
                self,
                "Dependencies installed",
                "Python dependencies from requirements.txt have been installed/updated.",
            )
        else:
            QMessageBox.critical(
                self,
                "Dependency error",
                "Failed to install some dependencies. See log for details.",
            )

    # ------------------------------------------------------------------
    # PAGE 5: Program update (git pull wrapper)
    # ------------------------------------------------------------------
    def _build_update_page(self) -> QWidget:
        page = QWidget()
        v = QVBoxLayout(page)

        title = QLabel("<h2>5. Check for Program Updates</h2>")
        title.setTextFormat(Qt.RichText)
        v.addWidget(title)

        desc = QLabel(
            "This step checks for newer versions of the ASV scripts from the private "
            "GitHub repository and pulls them if available."
        )
        desc.setWordWrap(True)
        v.addWidget(desc)

        btn_update = QPushButton("Check & Pull Latest ASV Version from GitHub")
        btn_update.clicked.connect(self.handle_update_repo)
        v.addWidget(btn_update)

        nav_layout = QHBoxLayout()
        nav_layout.addStretch()
        btn_next = QPushButton("Next ▶")
        btn_next.clicked.connect(lambda: self.stack.setCurrentIndex(3))
        nav_layout.addWidget(btn_next)
        v.addLayout(nav_layout)

        return page

    def handle_update_repo(self):
        """
        Wrapper around handle_clone_repo for the "Update" page;
        if the repo exists this effectively performs a 'git pull'.
        """
        if not self.repo_dir.exists():
            QMessageBox.warning(
                self,
                "Repository missing",
                "Clone the repository on the previous page before checking for updates.",
            )
            return
        self.handle_clone_repo()

    # ------------------------------------------------------------------
    # PAGE 6–7: Build with colcon, start physics & dashboard GUI
    # ------------------------------------------------------------------
    def _build_run_page(self) -> QWidget:
        page = QWidget()
        v = QVBoxLayout(page)

        title = QLabel("<h2>6–7. Build & Run Simulation</h2>")
        title.setTextFormat(Qt.RichText)
        v.addWidget(title)

        desc = QLabel(
            "From here you can build the ROS2 packages for the physics engine and "
            "web viewer, then start the physics engine and the dashboard GUI."
        )
        desc.setWordWrap(True)
        v.addWidget(desc)

        btn_build = QPushButton("Run colcon build (asv_digital_twin, asv_web_viewer)")
        btn_build.clicked.connect(self.handle_build_colcon)
        v.addWidget(btn_build)

        btn_start_phys = QPushButton("Start Physics Engine (ROS2 launch)")
        btn_start_phys.clicked.connect(self.handle_start_physics)
        v.addWidget(btn_start_phys)

        btn_start_gui = QPushButton("Start Dashboard GUI")
        btn_start_gui.clicked.connect(self.handle_start_dashboard)
        v.addWidget(btn_start_gui)

        return page

    def handle_build_colcon(self):
        """
        Run the colcon build command inside Ubuntu for the physics engine.
        """
        if not self.repo_dir.exists():
            QMessageBox.warning(self, "Repository missing", "Clone the repository first.")
            return

        wsl_repo = windows_to_wsl_path(str(self.repo_dir))
        cmd = (
            f"cd {wsl_repo} && "
            f"colcon build --packages-select asv_digital_twin asv_web_viewer"
        )
        self.log("Running colcon build inside Ubuntu...")
        res = run_wsl_cmd(cmd, distro=self.wsl_distro)
        self.log(res.stdout + "\n" + res.stderr)
        if res.returncode == 0:
            QMessageBox.information(self, "Build complete", "colcon build completed successfully.")
        else:
            QMessageBox.critical(self, "Build failed", "colcon build reported errors. See log.")

    def handle_start_physics(self):
        """
        Start the ROS2 launch file for the physics engine in a background WSL process.
        """
        if not self.repo_dir.exists():
            QMessageBox.warning(self, "Repository missing", "Clone the repository first.")
            return

        wsl_repo = windows_to_wsl_path(str(self.repo_dir))
        cmd = (
            f"cd {wsl_repo} && "
            f"source install/setup.bash && "
            f"ros2 launch asv_digital_twin master.launch.py"
        )
        self.log("Starting physics engine (ROS2 launch) in background WSL process...")
        subprocess.Popen(
            ["wsl", "-d", self.wsl_distro, "--", "bash", "-lc", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def handle_start_dashboard(self):
        """
        Start the PyQt dashboard GUI for the ASV digital twin in a background WSL process.
        """
        if not self.repo_dir.exists():
            QMessageBox.warning(self, "Repository missing", "Clone the repository first.")
            return

        wsl_repo = windows_to_wsl_path(str(self.repo_dir))
        cmd = (
            f"cd {wsl_repo} && "
            f"source install/setup.bash && "
            f"python3 src/asv_digital_twin/asv_digital_twin/main_app.py"
        )
        self.log("Starting dashboard GUI in background WSL process...")
        subprocess.Popen(
            ["wsl", "-d", self.wsl_distro, "--", "bash", "-lc", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # ------------------------------------------------------------------
    # Logging helper
    # ------------------------------------------------------------------
    def log(self, text: str):
        self.log_box.append(text)


def main():
    app = QApplication(sys.argv)
    win = LauncherWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

