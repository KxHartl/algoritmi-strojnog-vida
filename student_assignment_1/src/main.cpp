#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>

// --- Classic binary search on a sorted vector ---
// Returns the index of 'target' if found, otherwise -1.
int binarySearch(const std::vector<int>& arr, int target) {
    int left = 0;
    int right = arr.size() - 1;

    while (left <= right) {
        int mid = left + (right - left) / 2; // Avoids overflow
        if (arr[mid] == target) {
            return mid;
        } else if (arr[mid] < target) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    return -1;
}

int main() {
    // --- Initialize and sort the array ---
    std::vector<int> arr = {34, 7, 23, 32, 5, 62, 78, 1, 9, 12, 45, 67, 89, 21, 3, 8, 15, 27, 50, 40};
    std::sort(arr.begin(), arr.end());

    // --- Display the sorted array ---
    std::cout << "Sorted array: ";
    for (int num : arr) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    // --- Interactive search loop ---
    while (true) {
        std::cout << "Enter a number to search (or type 'exit' to quit): ";
        std::string input;
        std::getline(std::cin, input);

        if (input == "exit" || input == "quit") {
            break;
        }

        try {
            int target = std::stoi(input); // Convert input to integer
            int index = binarySearch(arr, target);
            if (index != -1) {
                std::cout << "Number found at index: " << index << std::endl;
            } else {
                std::cout << "Number not found in the array." << std::endl;
            }
        } catch (const std::invalid_argument& e) {
            std::cout << "Invalid input. Please enter a valid integer." << std::endl;
        } catch (const std::out_of_range& e) {
            std::cout << "Number out of range. Please enter a smaller integer." << std::endl;
        }
    }

    return 0;
}
