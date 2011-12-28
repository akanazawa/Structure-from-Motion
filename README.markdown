Affine Structure from Motion
----

A demo of the entire pipeline of basic affine structure from motion
including

1. keypoint selection `do_getKeypoints.m`
2. feature tracking `do_trackFeatures.m`
3. the factorization algorithm `do_factorization.m`

Specify experiment settings in the config file (location of the
images, data, etc), then in matlab do

`do_all('config')`


