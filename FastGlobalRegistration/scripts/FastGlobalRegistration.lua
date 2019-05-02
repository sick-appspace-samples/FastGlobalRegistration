--[[----------------------------------------------------------------------------
   
  Application Name:
  FastGlobalRegistration
          
  Summary:
  Register two point clouds and estimate a rigid transformation.
   
  Description:
  This samples loads point cloud data from a file and
   - creates a transformed version of the input cloud
   - smoothes surface with the Moving Least Squares method
   - computes surface normals
   - computes Fast Point Feature Histogram (FPFH) feature description
   - computes Fast Global Registration
   - transforms second cloud back to its original position
   
  How to run:
  This sample can be run on the emulator or any device with
  AppEngine version 2.6.0 or higher and supporting PointClouds.
  
  More Information:
  For more information on fast global registration see:
  http://vladlen.info/publications/fast-global-registration/
   
------------------------------------------------------------------------------]]

--Start of Global Scope---------------------------------------------------------

print("AppEngine version: " .. Engine.getVersion())

-- Path to test file
local FILE_PATH = "resources/flat_object.pcd"

-- Pausing duration for demonstration purpose only
local PAUSE = 2000

-- Setup viewer and decoration
local viewer = View.create()
viewer:setID("viewer3D")

local deco = View.PointCloudDecoration.create()
deco:setPointSize(1)
deco:setIntensityColormap(1)
viewer:setDefaultDecoration(deco)

-- Setup fast global registration
local reg = PointCloud.Registration.FastGlobal.create()
reg:setMaximumCorrespondenceDistance("RELATIVE", 0.025)
reg:setMaxIterations(500)
reg:setMaxTuples(1000)
reg:setSimilarityThreshold(0.95)
reg:setGraduatedNonConvexityFactor(1.4)

--End of Global Scope-----------------------------------------------------------

--Start of Function and Event Scope---------------------------------------------

local function viewPointCloud(pc)
  viewer:clear()
  viewer:addPointCloud(pc)
  viewer:present()
  Script.sleep(PAUSE) -- For demonstration purpose only
end

-- Entry point after Engine.OnStarted event
local function handleOnStarted()

  -- Load input point cloud
  local cloud1 = PointCloud.load(FILE_PATH)
  
  -- Create a ridgid transformation of the cloud (rotation + translation)
  local rigidTransform = Transform.createRigidAxisAngle3D({0.1, -0.2, 0.9}, 0.57, 10, 23, 80)
  local cloud2 = cloud1:transform(rigidTransform)
  
  viewPointCloud(cloud1)
  viewPointCloud(cloud2)
  
  -- Sample both clouds with a different number of points
  cloud1 = cloud1:sample(12000)
  cloud2 = cloud2:sample(15000)
  
  -- Remove outliers
  cloud1 = cloud1:filterStatisticalOutliers()
  cloud2 = cloud2:filterStatisticalOutliers()
  
  -- Smooth point clouds
  local tic = DateTime.getTimestamp()
  cloud1 = cloud1:filterMovingLeastSquares(10)
  cloud2 = cloud2:filterMovingLeastSquares(10)
  local toc = DateTime.getTimestamp()
  print("Smoothing both clouds took "..toc - tic.." ms")
  
  -- View both clouds
  viewPointCloud(cloud1)
  viewPointCloud(cloud2)
  
  -- Compute surface normals
  tic = DateTime.getTimestamp()
  local normals1 = cloud1:featureNormals(10, "RADIUS_SEARCH")
  local normals2 = cloud2:featureNormals(10, "RADIUS_SEARCH")
  toc = DateTime.getTimestamp()
  print("Computing the normals for both clouds took "..toc - tic.." ms")
  
  -- Compute the FPFH feature description used for the registration
  tic = DateTime.getTimestamp()
  local features1 = cloud1:featureFPFH(normals1, 10, "RADIUS_SEARCH")
  local features2 = cloud2:featureFPFH(normals2, 10, "RADIUS_SEARCH")
  toc = DateTime.getTimestamp()
  print("Computing the FPFH for both clouds took "..toc - tic.." ms")
  
  -- Compute the registration
  tic = DateTime.getTimestamp()
  local estimatedTransform = reg:compute(cloud1, features1, cloud2, features2)
  toc = DateTime.getTimestamp()
  print("Computing the registration took "..toc - tic.." ms")
  print("\nEstimated rigid transformation:")
  print(estimatedTransform:toString())
  
  -- Transform the second cloud back to its original position
  cloud2 = cloud2:transform(estimatedTransform)
  
  -- Merge the two clouds, setting a fix intensity for each of them
  local size1 = cloud1:getSize()
  local size2 = cloud2:getSize()
  local mergedCloud = PointCloud.create()
  for pos = 0, size1-1 do
    local x, y, z, _ = cloud1:getPoint(pos)
    mergedCloud:appendPoint(x, y, z, 0.0)
  end
  for pos = 0, size2-1 do
    local x, y, z, _ = cloud2:getPoint(pos)
    mergedCloud:appendPoint(x, y, z, 1.0)
  end
  
  -- View the merged cloud (each cloud is visualized in a different color)
  viewPointCloud(mergedCloud)
  
  print("App finished")
end
--The following registration is part of the global scope which runs once after startup
--Registration of the 'main' function to the 'Engine.OnStarted' event
Script.register("Engine.OnStarted", handleOnStarted)

--End of Function and Event Scope-----------------------------------------------
