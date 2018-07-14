detector = vision.CascadeObjectDetector('thermal.xml');
%%
% Read the test image.
img = imread('C:\Users\vc-lab\Desktop\dataset\jpg\301.jpg');
%%
% Detect a stop sign.
bbox = step(detector,img); 
%%
% Insert bounding box rectangles and return the marked image.
 detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'stop sign');
%%
% Display the detected stop sign.
figure; imshow(detectedImg);
%%
% Remove the image directory from the path.
rmpath(imDir); 
