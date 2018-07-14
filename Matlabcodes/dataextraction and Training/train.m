facepositive=mypositive(:,[1 3]);
negativeFolder='C:\Users\vc-lab\Desktop\dataset\negative';
trainCascadeObjectDetector('thermal.xml',facepositive, ...
negativeFolder,'FalseAlarmRate',0.1,'NumCascadeStages',4);
