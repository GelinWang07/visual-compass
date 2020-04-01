void generate3dCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr, cv::Mat depth, double camera_factor, double camera_cx, double camera_cy, double camera_fx, double camera_fy,float m_depth_threshold)
		{
			//pcl::console::TicToc tt;
			//std::cerr<<"start transforming from 2D to 3D \n",tt.tic();
			
			typedef pcl::PointXYZ PointT;
			cloudPtr.reset(new pcl::PointCloud<PointT>);
			for (int m = 0 ;m <depth.rows; m++)   //get every pixels from depth image
			{               
				for (int n = 0;n <depth.cols; n++)
				{            
					double d = depth.at<float>(m,n);
					PointT p;     
					
					if (d == 0 && d >= m_depth_threshold) 
					{
						//if d doesn't have value, or out of the range, then add p without nan
						p.x = std::numeric_limits<float>::quiet_NaN();
						p.y = std::numeric_limits<float>::quiet_NaN();
						p.z = std::numeric_limits<float>::quiet_NaN();
						
					}               
					else
					{
						p.z = double(d) / camera_factor;  //the transform formel
						p.x = (n - camera_cx) * p.z / camera_fx;
						p.y = (m - camera_cy) * p.z / camera_fy;

					}
					
					cloudPtr->points.push_back( p );                  //save p to the cloud

				}
			}
			//organized cloud
			cloudPtr->resize(depth.cols*depth.rows);
			cloudPtr->height = depth.rows;                                    
			cloudPtr->width = depth.cols;
			
			/*int w=cloudPtr->width;
			int h= cloudPtr->height;
			std::cerr<<"w :"<<w<<" h: "<<h<<std::endl;*/
			std::cerr<<"point cloud size before filtering = "<<cloudPtr->width*cloudPtr->height<<std::endl;
			cloudPtr->is_dense = false;

			pcl::PointCloud<PointT> cloud;
			cloud=*cloudPtr;

			if (cloud.isOrganized())
			{
				std::cerr<<"the cloud is organized" <<std::endl;
			}
	
			//std::cerr<<"end transforming from 2D to 3D: " << tt.toc()<<" ms\n";
		}
