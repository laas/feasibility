<p>Experimental project to learn a feasibility function, which directly relates two
points in configuration space</p>

<p>Sub-projects:
 -- Read in a .tris file, consisting of triangles, and display it in RVIZ + doing
 collision checking with FCL
 -- Random walk sampling of feasible positions between two objects
 -- log time for distance checking, depending on distance between objects</p>


<p>The usual path to a fresh build:</p>

<ul>
<li>$mkdir build
<li>$cd build
<li>$cmake ..
<li>$make

<li>$echo '/home/${MYNAME}/git/fastReplanningData/data/' > robotDATA.dat
</ul>
