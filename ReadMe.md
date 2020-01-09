# Function
This project is aimed to generate the datasets used in our paper "Geolocalization with aerial image sequence for UAVs". We generate aerial image sequences with the simulated flight of the UAVs with the cameras mounted to. For more details about how we do this, please refer to our article.

# Install
## 1. Install dependences
### (1) MRSID
The lib is used to decode the mrsid file, where the color aerial ortho imageries are saved, we provide the related lib files in the directory thrid_lib.

### (2) SHP
The lib is used to decode the shp file, we provide the related lib files in the directory thrid_lib.

### (3) opencv
Change the path for opencv to your own path.

### (4) Eigen3
Install the Eigen3 with the instruction in http://eigen.tuxfamily.org/index.php?title=Main_Page.

# Build
Build the project with the following commands:
$ cd ${root_path_of_the_project}

$ mkdir build

$ cd build

$ cmake ../

$ make

# Run
## 1. Data download
Download the color aerial ortho imageries from https://docs.digital.mass.gov/.
Download the road vector map file from OpenStreetMap, and save it in the form of .shp.

## 2. Set parameters
Set the parameters in the python script in 'tools/make_image_sequences.py' according to your requirements.


## 3. Run
Run the python script to generate the aerial image sequences using command:

$ python make_image_sequences.py
