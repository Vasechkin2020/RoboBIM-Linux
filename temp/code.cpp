
            // Если в кластере достаточно точек, сохраняем его
            if (cluster.size() >= MINPOINTS && cluster.size() < MAXPOINTS)
            {
                clusters.push_back(cluster);
                ROS_INFO(" clusters size %i: ", cluster.size());
            }
        }
        ROS_INFO("Grok Count clusters %i: ", clusters.size());
        return clusters;