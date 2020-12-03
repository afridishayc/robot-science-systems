
im1 = iread('rvc2_cover.png', 'double');
im2 = iread('rvc2.png', 'double');

sf1 = isurf(im1);
sf2 = isurf(im2);

m = sf1.match(sf2);

% idisp({im1, im2}, 'dark');
% m.subset(100).plot('w');

m.ransac(@fmatrix, 1e-4, 'verbose');
idisp({im1, im2});
m.inlier.subset(100).plot('g')
m.outlier.subset(100).plot('r')

[H,r] = m.ransac(@homography, 4);
idisp(im1)
plot_point(m.inlier.p1, 'ys')

