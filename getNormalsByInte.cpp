void getNormalsByInte(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr,pcl::PointCloud<pcl::Normal>::Ptr &NormalsPtr,float normal_smooth_size,float max_depth_change,int NE_method)
		{
			pcl::console::TicToc tt;
			std::cerr<<"start normal Integral imaging \n",tt.tic();
	
			//only for organized cloud
			NormalsPtr.reset(new pcl::PointCloud<pcl::Normal>);
			//std::cerr<<"start integral image normal calculating"<<std::endl;
			//create estimate
			 pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

			//three methods to estimate
			// ne.setRectSize(160,120);
			 switch (NE_method)
			 {
			 case 0:
				 ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
				 break;
			 case 1:
				 ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
				 break;
			 case 2:
				 ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
				 break;
			 default:
				 std::cerr<< "no avaliable estimation method used"<<std::endl;
				 break;
			 }
			 
			 ne.setInputCloud(cloudPtr);
			 ne.setNormalSmoothingSize(normal_smooth_size);
			 ne.setMaxDepthChangeFactor(max_depth_change); /// größerer wert!
			 ne.setDepthDependentSmoothing(true);
			 //ne.useSensorOriginAsViewPoint();
			 ne.compute(*NormalsPtr);
			 //std::cerr<<"normal estimation done"<<std::endl;

			 std::cerr<<"end normal Integral imaging: " << tt.toc()<<" ms\n";
		}
