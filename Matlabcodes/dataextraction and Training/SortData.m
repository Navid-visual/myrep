i=1;
for ii=1:8:8000

    temp=sprintf('%d.jpg',i+1);
    address{i}=['C:\Users\vc-lab\Desktop\dataset\jpg\' temp];
    txttemp=sprintf('%d.txt',i+1);
    txtmain{i}=['C:\Users\vc-lab\Desktop\dataset\txt1\bbox' txttemp];
fileID = fopen(txtmain{i},'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec,[4 4]);
    lefteye{i}=A(:,1)';
    righteye{i}=A(:,2)';
    nose{i}=A(:,3)';
    mouth{i}=A(:,4)';
    fclose all;
    i=i+1;
end

address=address';
lefteye=lefteye';
righteye=righteye';
nose=nose';
mouth=mouth';
    mypositive=table(address,lefteye,righteye,nose,mouth);