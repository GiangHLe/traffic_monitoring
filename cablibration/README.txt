Roadside camera calibration
---------------------------

If you find this software useful in your research, please cite:

DUBSKÁ Markéta, HEROUT Adam, JURÁNEK Roman and SOCHOR Jakub. Fully Automatic Roadside Camera Calibration for Traffic Surveillance. IEEE Transactions on Intelligent Transportation Systems. 2014, vol. 2014, no. 1, pp. 1-10. ISSN 1524-9050.

DUBSKÁ Markéta and HEROUT Adam. Real Projective Plane Mapping for Detection of Orthogonal Vanishing Points. In: Proceedings of BMVC 2013. Bristol: The British Machine Vision Association and Society for Pattern Recognition, 2013, pp. 1-10. ISBN 1-901725-49-9.

See AUTHORS and LICENSE

Requirements
------------
* Computer Vision System Toolbox
* Image Processing Toolbox
* NNet toobox (normr function only)
* Piotr's Toolbox (3.40+) https://github.com/pdollar/toolbox

Example
-------
[C,P,im] = calibRoad('video.avi','HorizonRange',[-4,4],'FocalRange',[400,600],'FinishAt',2000);

C   - calibration info
P   - Road plane parameters
im  - image with visualization of calibrated scene
