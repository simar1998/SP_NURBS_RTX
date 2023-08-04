//
// Created by simar on 7/21/2023.
//

#include <random>
#include "Analysis.h"

void Analysis::performAnalysis(MeshIntersect mesh) {
    std::vector<float> hilbertSpaceEmbeddings = embedTrianglesInHilbertSpace(mesh.getTriangles());
    // Perform k-means clustering with k = 3 (you can adjust k as needed)
    int k = 3;
    std::vector<int> clusterAssignments = kmeansClustering(hilbertSpaceEmbeddings, k,100);

    // Display the cluster assignments
    std::cout << "Cluster Assignments:" << std::endl;
    for (size_t i = 0; i < hilbertSpaceEmbeddings.size(); ++i) {
        std::cout << "Point " << i << ": Cluster " << clusterAssignments[i] + 1 << std::endl;
    }

}

std::vector<int> Analysis::kmeansClustering(const std::vector<float>& embeddings, int k, int maxIterations = 100) {
    std::vector<int> clusterAssignments(embeddings.size());

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(0, embeddings.size() - 1);

    // Initialize cluster centers with random data points
    std::vector<float> clusterCenters(k);
    for (int i = 0; i < k; ++i) {
        int randomIdx = distribution(gen);
        clusterCenters[i] = embeddings[randomIdx];
    }

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // Assign data points to the nearest cluster center
        for (size_t i = 0; i < embeddings.size(); ++i) {
            float minDistance = std::numeric_limits<float>::max();
            int clusterIdx = -1;
            for (int j = 0; j < k; ++j) {
                float distance = std::abs(embeddings[i] - clusterCenters[j]);
                if (distance < minDistance) {
                    minDistance = distance;
                    clusterIdx = j;
                }
            }
            clusterAssignments[i] = clusterIdx;
        }

        // Update cluster centers to be the mean of the data points in each cluster
        std::vector<float> clusterSums(k, 0.0f);
        std::vector<int> clusterSizes(k, 0);
        for (size_t i = 0; i < embeddings.size(); ++i) {
            int clusterIdx = clusterAssignments[i];
            clusterSums[clusterIdx] += embeddings[i];
            clusterSizes[clusterIdx]++;
        }

        for (int j = 0; j < k; ++j) {
            if (clusterSizes[j] > 0) {
                clusterCenters[j] = clusterSums[j] / clusterSizes[j];
            }
        }
    }

    return clusterAssignments;
}


//Using gausian curvature
float Analysis::calculateCurvature(const Analysis::Tri &triangle) {
    // Define the edges
    std::vector<float> edge1(3), edge2(3), edge3(3);
    for (int i = 0; i < 3; ++i) {
        edge1[i] = triangle.p1.values[i] - triangle.p0.values[i];
        edge2[i] = triangle.p2.values[i] - triangle.p1.values[i];
        edge3[i] = triangle.p0.values[i] - triangle.p2.values[i];
    }

    // Compute the cross products of the edges
    std::vector<float> cross1(3), cross2(3);
    cross1[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
    cross1[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
    cross1[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];

    cross2[0] = edge2[1] * edge3[2] - edge2[2] * edge3[1];
    cross2[1] = edge2[2] * edge3[0] - edge2[0] * edge3[2];
    cross2[2] = edge2[0] * edge3[1] - edge2[1] * edge3[0];

    // Compute the norms of the cross products
    float normCross1 = std::sqrt(cross1[0] * cross1[0] + cross1[1] * cross1[1] + cross1[2] * cross1[2]);
    float normCross2 = std::sqrt(cross2[0] * cross2[0] + cross2[1] * cross2[1] + cross2[2] * cross2[2]);

    // Compute the mean curvature
    float meanCurvature = 0.5f * (normCross1 + normCross2);

    return meanCurvature;
}


float Analysis::gaussianRBFKernel(float distance, float sigma) {
    return std::exp(-distance / (2 * sigma * sigma));
}

std::vector<float> Analysis::embedTrianglesInHilbertSpace(const std::vector<Tri> &triangles) {
    const float sigma = 1.0f; // Sigma parameter for the RBF kernel

    std::vector<float> embeddedPoints;

    for (const Tri& triangle : triangles) {
        // Calculate curvature for the triangle
        float curvature = calculateCurvature(triangle);

        // Use Gaussian RBF kernel to embed curvature into Hilbert space
        float embedding = gaussianRBFKernel(curvature, sigma);

        // Add the embedded point to the vector
        embeddedPoints.push_back(embedding);
    }

    return embeddedPoints;
}
