C++ demo code for the Poisson disk sampler in [geometry-central](https://geometry-central.net/surface/algorithms/surface_sampling/).

![gui screencap](https://raw.githubusercontent.com/nzfeng/poisson-disk-sampler-demo/main/media/gui_screencap.png)

The algorithm takes in a mesh and Poisson disk-samples the surface so that sampled points are distributed evenly **in 3D space**. Technically, Poisson disk sampling on surfaces should ensure that points are at least a certain *geodesic* distance away from each other. But for the purposes of visualization, it can be better to use extrinsic distance of the embedded surface instead, since this will ensure that the sampling **looks** even -- two points could have a large geodesic distance between them but be very near each other in 3D space, which might lead to a visually crowded image. Using 3D distance also speeds up the sampling process.

Currently the algorithm only works on manifold meshes.

The algorithm has a few parameters that roughly correspond to the algorithm of Bridson's 2007 [Fast Poisson Disk Sampling in Arbitrary Dimensions](https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf):

* `rCoef` corresponds to the minimum distance between samples, expressed as a multiple of the mean edge length. The actual minimum distance is computed as `r = rCoef * meanEdgeLength`.
* `kCandidates` = the number of candidate points chosen from the (r,2r)-annulus around each sample.

Additionally, you can specify points around samples should be avoided. By default, samples will avoid these points with the same radius `r` used in the rest of the algorithm. You can optionally specify a "radius of avoidance" for these points, where the radius of avoidance is given in multiples of `r`. The radius of avoidance can be further be specified to be a radius in 3D space, or a radius in terms of distance along the surface. The former will produce a radius of avoidance that will appear perfectly round and is likely more visually pleasing, but for very large radii may occlude samples from opposite sides of the mesh. The latter will restrict the radius of avoidance to only be along the surface, but such a metric ball will not appear perfectly round, especially in areas with very large and sudden changes in curvature.

## Cloning and building

```sh
git clone --recursive https://github.com/nzfeng/poisson-disk-sampler-demo.git
cd poisson-disk-sampler-demo
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
./bin/poisson_disk /path/to/your/mesh.obj
```