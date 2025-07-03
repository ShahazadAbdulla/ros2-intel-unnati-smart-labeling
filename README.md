# Smart Product Labeling & Traceability System (ROS 2)

This project, developed for the **Intel® Unnati Industrial Training Program**, is a comprehensive, automated system that simulates an industrial quality control station. It uses a modular ROS 2 architecture to validate product labels through a multi-stage AI pipeline, provides a real-time 3D simulation with PyBullet, and ensures complete traceability via a persistent SQLite database.

**Team:** Robotrace  
**Author(s):** Shahazad Abdulla, Hridya Mariam Reji

---

## Video Demonstration

Before you begin, see the complete system in action!

-  **Watch the Full System Demo Videos**: https://drive.google.com/drive/folders/1eDimLPakpihkVoP1BTZO0_w6PpfFhtQ_?usp=sharing

---

## Key Features

- **Modular ROS 2 Architecture**: A distributed system of 9 communicating nodes for scalability and clear separation of concerns.

- **PyBullet Digital Twin**: A real-time 3D visualization of the conveyor, products, and a robotic rejector arm.

- **Comprehensive AI/CV Validation Pipeline**:
  - **Compliance & Quality**: Checks for RoHS compliance and image blur (Laplacian variance).
  - **QR Code Validation**: Decodes QR codes (`pyzbar`) and verifies serial number content.
  - **OCR Validation**: Reads text (`easyocr`), then cleans and validates Batch ID and Serial Number using custom logic.
  - **ML-Powered Print Quality Assessment**: A custom-trained `TensorFlow/Keras` model classifies label print quality as `"GOOD"` or `"BAD"`.

- **Interactive Tkinter UI**: For system control (**START/EXIT**), real-time status monitoring, and an in-app traceability report viewer.

- **Persistent Traceability**: All validation results are logged to a robust SQLite database that is saved to your host machine.

- **Dockerized for Portability**: The entire application and its complex environment are containerized with Docker for one-command setup and execution.

---

## System Architecture

The system is built on a computation graph of ROS 2 nodes that manage the flow of data and commands. A `conductor_node` orchestrates the process, driven by the `ui_node`. The `pybullet_visualizer_node` provides the visual simulation. The `ai_validation_node` performs the core logic, calling the `label_quality_ml_node` service and publishing results that are logged by the `database_logger_node`.

![Alt text](docs/SystemArchitecture.png#width=50%)

---

## Getting Started: Setup & Execution

This project is designed to be built and run inside a Docker container. This ensures all dependencies are handled correctly.

### Prerequisites

- A **Linux host machine** (Ubuntu 22.04 recommended).
- **Docker Engine** installed and running.
- An **active internet connection** for the initial build.
- For GUI visualization, an **X11 server** is required (standard on most Linux desktops).

---

### Step 1: Clone the Repository

```bash
git clone https://github.com/ShahazadAbdulla/ros2-intel-unnati-smart-labeling.git
cd ros2-intel-unnati-smart-labeling
```
### Step 2: Download and Place Project Assets

The image datasets and trained ML models are hosted separately due to their size.

1.  **Download the project\_assets.zip file** 
- Download `project_assets.zip` from this https://drive.google.com/file/d/1ASYQg9XSTQtSha_tagggE2zGxNKZx_I3/view?usp=sharing
    
2.  **Unzip the file.** You will get three folders: label\_images, label\_images\_test, and saved\_models.

3.  **Place the folders as shown below**
    
```bash
ros2-intel-unnati-smart-labeling/
├── Dockerfile
├── README.md
└── src/
    └── smart_labeling_system_pkg/
        ├── config/
        ├── label_images/       <-- You placed this here
        ├── label_images_test/  <-- You placed this here
        └── saved_models/       <-- You placed this here
```
    
### Step 3: Build the Docker Image

From the root of the project directory (ros2-intel-unnati-smart-labeling/), run the build command. This will take a significant amount of time on the first run.

```bash
docker build -t smart_labeling_system .
```

### Step 4: Run the System

The system uses a persistent volume to save the traceability.db file to your host machine.

1.  **Create the persistent data directory** on your host:
```bash
mkdir -p persistent_data/database
``` 
2. **(For GUI Mode) Allow your display server to accept connections from the container:**
```bash      
xhost +
```
3. **Run the container** This command launches the full simulation with the smaller test dataset.
```bash      
docker run -it --rm --name smart_labeling_run --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${PWD}/persistent_data/database:/ros2_ws/install/smart_labeling_system_pkg/share/smart_labeling_system_pkg/database \
    smart_labeling_system
```
  - The Tkinter UI and PyBullet simulation windows should appear.
  - Use the UI to "START" and "EXIT" the system.
  - Click "TRACEABILITY REPORT" in the UI to view the live database log within the application.
    

### Other Launch Options

You can override the default launch arguments to run different configurations. Append the launch command to the docker run ...:
- **Run with the full 50-product dataset in headless (no-GUI) mode for a quick data-processing run:**
```bash
docker run -it --rm --name smart_labeling_run --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${PWD}/persistent_data/database:/ros2_ws/install/smart_labeling_system_pkg/share/smart_labeling_system_pkg/database \
    smart_labeling_system ros2 launch smart_labeling_system_pkg system_launch.py sim_mode:=false csv_file:=products.csv images_folder:=label_images/
```
    

### Accessing the Final Database

After running the system, the traceability log will be available on your host machine at:
./persistent_data/database/traceability.db

You can open this file with any local SQLite browser for further analysis.

Challenges & Future Work
------------------------

This project involved overcoming several real-world distributed systems challenges, including managing node lifecycles cleanly, preventing race conditions through careful event-driven design, and solving dependency conflicts in a containerized environment. Future enhancements could include implementing ROI-based OCR for higher accuracy, training a dedicated product surface defect model, and deploying the system on physical edge-computing hardware.
