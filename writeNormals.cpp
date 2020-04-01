void writeNormals(pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_Ptr, std::string txtpath)
			{
				//todo:: a stream to write
				typedef pcl::Normal PointN;
				std::ofstream outfile;
				outfile.open(txtpath, ios::trunc);
				for (size_t i = 0; i < cloud_normals_Ptr->points.size(); i++)
				{
					outfile <<" height: "<<cloud_normals_Ptr->height <<" width: "<<cloud_normals_Ptr->width
							<<" i: "<< i
							<<" nx: "<< cloud_normals_Ptr->points[i].normal_x
							<<" ny: "<< cloud_normals_Ptr->points[i].normal_y
							<<" nz: "<< cloud_normals_Ptr->points[i].normal_z
							<<" curvature: "<< cloud_normals_Ptr->points[i].curvature
							<<"\n";
				}
				outfile.close();
				std::ifstream infile;
				infile.open(txtpath);

			}
