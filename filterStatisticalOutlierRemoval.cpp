void filterStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,pcl::PointCloud<pcl::Normal>::Ptr &in_normals_ptr,pcl::PointCloud<pcl::Normal>::Ptr &out_normals_ptr,std::vector<int> &out_cloud_indices,int MeanK,float stddev)
		{
			//pcl::console::TicToc tt;
			//std::cerr<<"start statistical outlier removel filtering \n",tt.tic();
			
			typedef pcl::PointXYZ PointT;
			
			//creat the filtering object
			pcl::StatisticalOutlierRemoval<PointT> sor;
			sor.setInputCloud(in_cloud_ptr);
			sor.setMeanK(MeanK);
			sor.setStddevMulThresh(stddev);

			out_cloud_ptr.reset(new pcl::PointCloud<PointT>);
			sor.filter(*out_cloud_ptr);
			out_normals_ptr.reset(new pcl::PointCloud<pcl::Normal>);
			sor.filter(out_cloud_indices);
			pcl::copyPointCloud(*in_normals_ptr,out_cloud_indices,*out_normals_ptr);
			//output and write
			std::cerr<<"Point cloud after Statistical Outlier Removal filtering: "<<out_cloud_ptr->width*out_cloud_ptr->height<< "data points ("<< pcl::getFieldsList(*out_cloud_ptr)<<")."<<std::endl;
			//std::cerr<<"end statistical outlier removel filtering: " << tt.toc()<<" ms\n";

		}
