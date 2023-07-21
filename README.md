Non-Planar 3D Printing: GCode Generation with Ray Tracing, Data Science, and NURBS
3D-Printing

This project aims to revolutionize 3D printing technology by generating non-planar GCode using ray tracing, data science, and Non-Uniform Rational B-Splines (NURBS) techniques. 
Unlike conventional 3D printing, which utilizes planar layers, our approach creates smooth, curved layers, resulting in a 
higher-quality 3D print with better surface finishing and increased structural integrity. Opening up additional opportunisties in the 3d printing 
capabilites, and hopefully allowing designers to make parts with funtianlities that are not yet at the time of writing possible. 


Notes: I am currenlty a bit tilted but will try to work on some cool ideas
- The following section is rough notes if you will. The goal is to conjure ideas to determining logic for non plannar gcode generation.

  - A method woud be to perform z axis intersections on the mesh at a pre determinied z sampling rate. 
  Each intersect value will be stored in together in a single giant z axis intersect points list with the indicies of each layer data stored in a s
  seperate list. The indicies will determine the location of the layers within the large list. This way performing operations on the  total
  sum of the permiter indexes. I will have to figure out methods of determining the internal structure of the mesh using permiter values 
  and to clasify internal permiters. This might be a very potent technique to determine internal or complex multi bi-sectional perimeter meshes.
  I will be having to come up with a series of algorithims that perform iterative operations on the sum total of these interectional values. 
  These iterative linear systems and vector calculations and operations will determine the eligibility of a surface for non-planar movement.
  I will also need to store the value of the primitive at each iteration point. This I believe can be useful in determining the internal 
  geometry associated with that particular region. 
  

