
#include "kdtree.h"

// std::vector<int> medianSort(const std::vector<int> &vec);
// std::vector<int> medianCombine(const std::vector<int> &xind, const std::vector<int> &yind, const std::vector<int> &zind);

std::vector<int> medianSort(const std::vector<int> &vec)
{
	// Out-to-middle algorithm (could rewrite as middle-out)
	// Starts with numbers furthest from median, ends with median number
	std::vector<int> nvec;
	nvec.reserve(vec.size()); // Pre-allocate memory

	int N = vec.size();
	for (int i = 0; i < N/2; i++)
	{
		nvec.push_back(vec[i]);       // Iterate forward
		nvec.push_back(vec[(N-1)-i]); // Iterate backwards
	}
	// If length is even, sizes will be equal
	// If length is odd, need to add median to end as a seperate step
	if (nvec.size() != vec.size())
	{
		nvec.push_back(vec[N/2]);
	}
	return nvec;
}

std::vector<int> medianCombine(const std::vector<int> &xind, const std::vector<int> &yind, const std::vector<int> &zind)
{
	int N = xind.size();
	std::vector<bool> isProcessed(N,false);
	std::vector<int> ind;
	ind.reserve(N); // Pre-allocate memory

	for (int i = 0; ind.size() < N; i++)
	{
		// Since median is at the end, we iterate in reverse
		int xi = xind[(N-1)-i];
		if (~isProcessed[xi]){
			isProcessed[xi] = true;
			ind.push_back(xi);
		}
		int yi = yind[(N-1)-i];
		if (~isProcessed[yi]){
			isProcessed[yi] = true;
			ind.push_back(yi);
		}
        int zi = zind[(N-1)-i];
		if (~isProcessed[zi]){
			isProcessed[zi] = true;
			ind.push_back(zi);
		}
	}
	return ind;
}

std::vector<int> medianSortPoints(const std::vector<std::vector<float>> &points)
{
	// Initialize arrays
	std::vector<int> xind(points.size()); // ids sorted by x-axis
	std::vector<int> yind(points.size()); // ids sorted by y-axis
    std::vector<int> zind(points.size()); // ids sorted by y-axis

	// Set to be array of indices 0...n
	iota(xind.begin(), xind.end(), 0);
	iota(yind.begin(), yind.end(), 0);
    iota(zind.begin(), zind.end(), 0);

	// Sort by axis
	std::sort(xind.begin(), xind.end(), [&points](int i, int j) {return points[i][0] < points[j][0];});
	std::sort(yind.begin(), yind.end(), [&points](int i, int j) {return points[i][1] < points[j][1];});
    std::sort(zind.begin(), zind.end(), [&points](int i, int j) {return points[i][2] < points[j][2];});

	// medianSort returns array that starts with numbers furthest from median, ends with median number
	xind = medianSort(xind);
	yind = medianSort(yind);
    zind = medianSort(zind);

	// Combine median indices
	std::vector<int> ind = medianCombine(xind,yind,zind);
	return ind;
}
