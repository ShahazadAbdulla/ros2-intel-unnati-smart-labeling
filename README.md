# Smart Product Labeling & Traceability System (ROS 2)

This project, developed for the **Intel® Unnati Industrial Training Program**, is a comprehensive, automated system that simulates an industrial quality control station. It uses a modular ROS 2 architecture to validate product labels through a multi-stage AI pipeline, provides a real-time 3D simulation with PyBullet, and ensures complete traceability via a persistent SQLite database.

**Team:** Robotrace  
**Author(s):** Shahazad Abdulla, Hridya Mariam Reji

---

## Video Demonstration & Project Assets

Before you begin, see the complete system in action!

-  **Watch the Full System Demo Videos**: https://drive.google.com/drive/folders/1eDimLPakpihkVoP1BTZO0_w6PpfFhtQ_?usp=sharing
-  **Download Required Assets**: You will need to download the image datasets and the trained ML model.

  - Download `project_assets.zip` from this [Google Drive Link](#)
  - (See **Step 2** in the Setup Instructions for where to place these files.)

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

> **Note:** This is a placeholder link. You must create a `docs` folder in your repo, upload your diagram as `system_architecture.png`, commit, push, and then update this link.

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
cd ros2_intel_ws
```
### Step 2: Download and Place Project Assets

The image datasets and trained ML models are hosted separately due to their size.

1.  **Download the project\_assets.zip file** using the Google Drive link provided at the top of this README.
    
2.  **Unzip the file.** You will get three folders: label\_images, label\_images\_test, and saved\_models.
    
3.  Generated code ros2-intel-unnati-smart-labeling/├── Dockerfile├── README.md└── src/ └── smart\_labeling\_system\_pkg/ ├── config/ ├── label\_images/ <-- You placed this here ├── label\_images\_test/ <-- You placed this here └── saved\_models/ <-- You placed this here IGNORE\_WHEN\_COPYING\_START
    
4.  Use code [with caution](https://support.google.com/legal/answer/13505487). IGNORE\_WHEN\_COPYING\_END
    

### Step 3: Build the Docker Image

From the root of the project directory (ros2-intel-unnati-smart-labeling/), run the build command. This will take a significant amount of time on the first run.

Generated bash

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML      `docker build -t smart_labeling_system .`    

IGNORE\_WHEN\_COPYING\_START Use code [with caution](https://support.google.com/legal/answer/13505487). BashIGNORE\_WHEN\_COPYING\_END

### Step 4: Run the System

The system uses a persistent volume to save the traceability.db file to your host machine.

1.  Generated bash mkdir -p persistent\_data/database IGNORE\_WHEN\_COPYING\_START
    

*   Use code [with caution](https://support.google.com/legal/answer/13505487). BashIGNORE\_WHEN\_COPYING\_END
    
*   Generated bash xhost + IGNORE\_WHEN\_COPYING\_START
    
*   Use code [with caution](https://support.google.com/legal/answer/13505487). BashIGNORE\_WHEN\_COPYING\_END
    
*   Generated bash docker run -it --rm --name smart\_labeling\_run --privileged \\ -e DISPLAY=$DISPLAY \\ -v /tmp/.X11-unix:/tmp/.X11-unix \\ -v ${PWD}/persistent\_data/database:/ros2\_ws/install/smart\_labeling\_system\_pkg/share/smart\_labeling\_system\_pkg/database \\ smart\_labeling\_system IGNORE\_WHEN\_COPYING\_START
    

1.  Use code [with caution](https://support.google.com/legal/answer/13505487). BashIGNORE\_WHEN\_COPYING\_END
    
    *   The Tkinter UI and PyBullet simulation windows should appear.
        
    *   Use the **UI** to "START" and "EXIT" the system.
        
    *   Click "TRACEABILITY REPORT" in the UI to view the live database log within the application.
        

### Other Launch Options

You can override the default launch arguments to run different configurations. Append the launch command to the docker run ... command.

*   Generated bash docker run -it --rm ... smart\_labeling\_system ros2 launch smart\_labeling\_system\_pkg system\_launch.py sim\_mode:=false csv\_file:=products.csv images\_folder:=label\_images/ IGNORE\_WHEN\_COPYING\_START
    
*   Use code [with caution](https://support.google.com/legal/answer/13505487). BashIGNORE\_WHEN\_COPYING\_END
    

### Accessing the Final Database

After running the system, the traceability log will be available on your host machine at:./persistent\_data/database/traceability.db

You can open this file with any local SQLite browser for further analysis.

Challenges & Future Work
------------------------

This project involved overcoming several real-world distributed systems challenges, including managing node lifecycles cleanly, preventing race conditions through careful event-driven design, and solving dependency conflicts in a containerized environment. Future enhancements could include implementing ROI-based OCR for higher accuracy, training a dedicated product surface defect model, and deploying the system on physical edge-computing hardware.
