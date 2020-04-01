bool segmentPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloudPtr, pcl::PointCloud<pcl::Normal>::Ptr &normalsPtr,pcl::ModelCoefficients::Ptr &coefficients_normals_ptr,pcl::PointIndices::Ptr &inliers_plane_Ptr,
			double threshold_in_degree,bool optimize,float ransac_probability,bool sac_method)
		{
			//pcl::console::TicToc tt;
			typedef pcl::PointXYZ PointT;
			typedef pcl::Normal PointN;
			pcl::SACSegmentationFromNormals<PointT, PointN>seg;
			//std::cerr<<"start ransac segmentation\n",tt.tic();

			if (in_cloudPtr->size()>=1000)
			{
				//list of parameters:
				double NormalDistanceWeight;
				double threshold;
				NormalDistanceWeight = 1.0;
				threshold = threshold_in_degree*M_PI/180;

				//create the segmantation object for the model and set all the parameters
				seg.setInputCloud(in_cloudPtr);
				seg.setInputNormals(normalsPtr);
				seg.setNormalDistanceWeight(NormalDistanceWeight);
				seg.setOptimizeCoefficients(optimize);
				seg.setProbability(ransac_probability);
				//seg.initSACModel(pcl::SACMODEL_NORMAL_PLANE);
				seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);

				if (sac_method)
				{
					seg.setMethodType(pcl::SAC_RANSAC);
				}
				else
				{
					seg.setMethodType(pcl::SAC_PROSAC);
				}
			
				seg.setDistanceThreshold(threshold);
				
				//Dataset
				coefficients_normals_ptr.reset(new pcl::ModelCoefficients);
				inliers_plane_Ptr.reset(new pcl::PointIndices);
		
				//std::cerr<<"Starting ransac for plane using normals"<<std::endl;
				seg.segment(*inliers_plane_Ptr, *coefficients_normals_ptr);

				std::cerr<< "Model Coefficients:  " << coefficients_normals_ptr->values[0] <<" "
													<< coefficients_normals_ptr->values[1] <<" "
													<< coefficients_normals_ptr->values[2] <<" "
													<< coefficients_normals_ptr->values[3] <<" "<<std::endl;
				//std::cerr<<"end ransac segmentation: " << tt.toc()<<" ms\n";
		
				return true;
			}
			else
			{
				//std::cerr<<"end ransac segmentation: " << tt.toc()<<" ms\n";

				return false;
			}
			
		}
