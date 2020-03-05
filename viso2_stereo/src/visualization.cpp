// TODO migrate

#ifdef DBG_CREATE_VISUALIZATION_IMAGES
        cv_bridge::CvImagePtr pImgL = cv_bridge::toCvCopy(
          l_image_msg,
          sensor_msgs::image_encodings::RGB8);

        cv::Point2f pt1;
        cv::Point2f pt2;
        cv::Point2f ptd;

        float ptDist = 0.0f;

        Matcher::p_match goodMatch;

        cv::Mat pxVal(1, 1, CV_8UC1, cv::Scalar(0));
        cv::Mat pxRGB(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));

        if (success) {
          // Plot all matches
          for (u_int32_t i = 0; i < matches.size(); ++i) {
            goodMatch = matches[i];

            pt1.x = goodMatch.u1p;  pt1.y = goodMatch.v1p;
            pt2.x = goodMatch.u1c;  pt2.y = goodMatch.v1c;

            line(pImgL->image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
            line(pImgL->image, pt1, pt1, cv::Scalar(255, 0, 0), 5);
          }

          // Plot inliers
          for (u_int32_t i = 0; i < inlier_indices.size(); ++i) {
            goodMatch = matches[inlier_indices[i]];

            pt1.x = goodMatch.u1p;      pt1.y = goodMatch.v1p;
            pt2.x = goodMatch.u1c;      pt2.y = goodMatch.v1c;

            // Colormap
            ptd = pt2 - pt1;
            ptDist = sqrtf(ptd.x * ptd.x + ptd.y * ptd.y);
            ptDist = rintf(ptDist * (84.0f / 35.0f));
            pxVal.at<uchar>(0, 0) = 84 - (ptDist <= 84 ? ptDist : 84);
//						ROS_DEBUG("%.1f, %d", ptDist, pxVal.at<uchar>(0,0));
            cv::applyColorMap(pxVal, pxRGB, cv::COLORMAP_HSV);
//						ROS_DEBUG("%d, %d, %d", pxRGB.at<cv::Vec3b>(0, 0)[0], pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]);

            line(
              pImgL->image, pt1, pt2, cv::Scalar(
                pxRGB.at<cv::Vec3b>(0, 0)[0],
                pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]), 2);
            line(
              pImgL->image, pt1, pt1, cv::Scalar(
                pxRGB.at<cv::Vec3b>(0, 0)[0],
                pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]), 5);
          }
        }

        imshow("matches", pImgL->image);
        cv::waitKey(1);

  #ifdef DBG_CREATE_VISUALIZATION_IMAGES_PATH
        // Write the image to disk
        std::stringstream fnStream("");
        fnStream << std::string(DBG_CREATE_VISUALIZATION_IMAGES_PATH) << std::string("/img_");
        fnStream << std::setw(8) << std::setfill('0') << l_image_msg->header.seq << ".png";
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        cv::imwrite(fnStream.str(), pImgL->image, compression_params);
    #endif
#endif