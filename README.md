<h2> Fast computation of feasibility for contact points </h2>
<p>Experimental project to learn a feasibility function, which directly relates two
points in configuration space</p>

<p>Sub-projects:
 -- Read in a .tris file, consisting of triangles, and display it in RVIZ + doing
 collision checking with FCL
 -- Random walk sampling of feasible positions between two objects
 -- log time for distance checking, depending on distance between objects</p>

<h2> Dependencies </h2>
<p> Fast Replanner -- ssh://<name>@trac.laas.fr/git/jrl/rblink/fast-replanning</p>
<p> FCL lib -- git://github.com/flexible-collision-library/fcl.git</p>

<p>The usual path to a fresh build:</p>

<ul>
<li>echo '/home/${MYNAME}/git/fastReplanningData/data/' > robotDATA.dat
<li>mkdir build
<li>cd build
<li>cmake ..
<li>make
</ul>
