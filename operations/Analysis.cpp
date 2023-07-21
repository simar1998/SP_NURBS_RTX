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



float Analysis::calculateCurvature(const Analysis::Tri &triangle) {
    // Perform your curvature calculation here
    // For this example, let's assume we calculate the average of the three edge lengths
    float edge1 = std::sqrt((triangle.p0.values[0] - triangle.p1.values[0]) * (triangle.p0.values[0] - triangle.p1.values[0]) +
                            (triangle.p0.values[1] - triangle.p1.values[1]) * (triangle.p0.values[1] - triangle.p1.values[1]) +
                            (triangle.p0.values[2] - triangle.p1.values[2]) * (triangle.p0.values[2] - triangle.p1.values[2]));
    float edge2 = std::sqrt((triangle.p1.values[0] - triangle.p2.values[0]) * (triangle.p1.values[0] - triangle.p2.values[0]) +
                            (triangle.p1.values[1] - triangle.p2.values[1]) * (triangle.p1.values[1] - triangle.p2.values[1]) +
                            (triangle.p1.values[2] - triangle.p2.values[2]) * (triangle.p1.values[2] - triangle.p2.values[2]));
    float edge3 = std::sqrt((triangle.p2.values[0] - triangle.p0.values[0]) * (triangle.p2.values[0] - triangle.p0.values[0]) +
                            (triangle.p2.values[1] - triangle.p0.values[1]) * (triangle.p2.values[1] - triangle.p0.values[1]) +
                            (triangle.p2.values[2] - triangle.p0.values[2]) * (triangle.p2.values[2] - triangle.p0.values[2]));

    return (edge1 + edge2 + edge3) / 3.0f; // Average edge length as an example of curvature


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
