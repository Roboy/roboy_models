# Robot Models
Models of Roboy and related robots consisting of meshes and sdfs.

New models can be created for example from Autodesk Fusion CAD files with the SDFusion exporter (https://github.com/Roboy/SDFusion). Models are used for physics Simulation and are pulled from Unity for visualization in Virtual Environment.



## Requirements for creating a new model
Due to the multifold use of the models, please consider common requirements for model generation:



Files:
- .STL and additional .DAE file extensions for the modelparts plus a .SDF model file to determine the joints, etc.

Component Naming:
- destinct, for example it should be avoided to name parts like thigh1/ 2, they rather should be thigh_left/ _right.
- simple and intuitive (no special characters!), for example no left_hand1v(0.2)1, but rahter left_hand.

Scaling:
- consistent within and between models, e.g. roboy's hand should not be as big as the complete roboy model.

Symmetry:
- homogenous distribution of components, this means if the model has a symmetric characteristic, the distribution in between the seperate areas should be homogenous. 
For example the model of the left hand should not contain in addition the lower arm, if the right model leaves it out. So the Seperation should be equal (not different!) pasted in the models directory.

Model Size:
- Due to simulation performance try to decrease the model size as much as possible, e.g. by using Blender's Polygon reduction tools

Description:
- Add a proper description into the model.config file, that especially describes the purpose and usage of the model as well as applied modification techniques. Also add your author name to allow further questions and a thumbnail file thats shows the robot.

Folder Structure:
- /model_name
  -  model.sdf<br />
  - model.config<br />
  - thumbnail.png<br />
  - /meshes<br />
	  - /visual<br />
	      - VIS_linkX.dae<br />
	      - ...<br />
	   - /collision<br />
	      - COL_linkX.dae<br />
	      - ...<br />



### this repo uses [git-lfs](https://git-lfs.github.com/) for handling the largs stl files
