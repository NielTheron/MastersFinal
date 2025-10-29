function SaveSatelliteImages(satellite_image,r)
    filename = sprintf('sat_image_%03d.png', r); 
    outputFolder = 'C:\Users\Niel\Desktop\1. Masters\1. Satellite\2. Image Generator\SatelliteImages';
    fullpath = fullfile(outputFolder,filename);
    imwrite(satellite_image,fullpath);
end

