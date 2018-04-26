/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

 #ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
 #define __VELODYNE_POINTCLOUD_POINT_TYPES_H
 #define PCL_NO_PRECOMPILE
 #include <pcl/point_types.h>
 
 #define PCL_ADD_UNION_OFFSET4D \
   union EIGEN_ALIGN16 { \
     float data_o[4]; \
     float offset[3]; \
     struct { \
       float offset_x; \
       float offset_y; \
       float offset_z; \
     }; \
   };
 
 #define PCL_ADD_EIGEN_MAPS_OFFSET4D \
   inline pcl::Vector3fMap getOffsetVector3fMap() {\
     return (pcl::Vector3fMap(data_o)); } \
   inline pcl::Vector3fMapConst getOffsetVector3fMap() const {\
     return (pcl::Vector3fMapConst(data_o)); } \
   inline pcl::Vector4fMap getOffsetVector4fMap() {\
     return (pcl::Vector4fMap(data_o)); } \
   inline pcl::Vector4fMapConst getOffsetVector4fMap() const {\
     return (pcl::Vector4fMapConst(data_o)); } \
   inline pcl::Array3fMap getOffsetArray3fMap() {\
     return (pcl::Array3fMap(data_o)); } \
   inline pcl::Array3fMapConst getOffsetArray3fMap() const {\
     return (pcl::Array3fMapConst(data_o)); } \
   inline pcl::Array4fMap getOffsetArray4fMap() {\
     return (pcl::Array4fMap(data_o)); } \
   inline pcl::Array4fMapConst getOffsetArray4fMap() const {\
     return (pcl::Array4fMapConst(data_o)); }
 
 #define PCL_ADD_OFFSET4D \
   PCL_ADD_UNION_OFFSET4D \
   PCL_ADD_EIGEN_MAPS_OFFSET4D
 
 namespace velodyne_pointcloud
 {
   /** Euclidean Velodyne coordinate, including intensity, ring number and segmentation label. */
 struct PointOffsetIRL {
     PCL_ADD_POINT4D;                    // quad-word XYZ
     PCL_ADD_OFFSET4D;
     float    intensity;                 ///< laser intensity reading
     uint16_t ring;                      ///< laser ring number
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
 } EIGEN_ALIGN16;
 
 };  // namespace velodyne_pointcloud
 
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointOffsetIRL,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, offset_x, offset_x)
                                   (float, offset_y, offset_y)
                                   (float, offset_z, offset_z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring))
 
 #endif  // __VELODYNE_POINTCLOUD_POINT_TYPES_H
 