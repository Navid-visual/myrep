atPath = getenv('FLIR_Atlas_MATLAB');
atLive = strcat(atPath,'Flir.Atlas.Live.dll');
asmInfo = NET.addAssembly(atLive);
%init camera discovery
test = Flir.Atlas.Live.Discovery.Discovery;
disc = test.Start(10);
handles.disc = disc;
for i=1:10
ImStream = Flir.Atlas.Live.Device.ThermalCamera(true);    
    ImStream.Connect(disc.Item(3));
    %save the stream
    handles.ImStream = ImStream;
    %set the Iron palette
    pal = ImStream.ThermalImage.PaletteManager;
    ImStream.ThermalImage.Palette = pal.Iron; 
    
   
    
      img = ImStream.ThermalImage.ImageArray;
      %convert to Matlab type
      X = uint8(img);
      %show image with Matlab imshow
      imshow(X);
      drawnow
      
end
ImStream = handles.ImStream;
ImStream.Disconnect();
ImStream.Dispose();