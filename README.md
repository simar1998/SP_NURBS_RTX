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
  - I need to make a mesh deformation technique based on ray intersection tests from the z axis plane towards the top surface of the mesh whose surface
  may be requiring non planar motion to to navigate. I propose a method from which I can Perform intersection tests and calculate the lower bounds of a
  subset of the mesh that I can use to perform the possions surface reconstruction algorithim to generate a mesh that represents
  a subset of the original mesh that will indicate a complex 3c crosssection of the orignal mesh. When this secondary mesh is generated I will 
  perform a boolean operation on the set mesh with the subset mesh to create a mesh without the subset cross section. With this I can perform 
  furthur ray intersection tests that can provide me the points in which the toolpath must navigate to in order to perform successful non planar 
  motion. Using this approach of iteratively performing boolean operations on the mesh and performing ray intersection test and providing
  some sort of decision logic that determines the lower z axis bound of the ray intersection points generated I think I can generate a non 
  planar tool path generation engine for FFF 3D printers. 
  
***** Project rebased onto https://github.com/simar1998/SculptPath ******
