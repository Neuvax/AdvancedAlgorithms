/*
A01640826 |Jorge German Wolburg Trujillo
Activity 1.1 "Divide and Conquer" Implementation
*/

#include <iostream>
#include <vector>

using namespace std;

// Here we grab both halves of the array and merge them in descending order
void merge(vector<double>& arr, int left, int mid, int right) {
    int n1 = mid - left + 1;
    int n2 = right - mid;

    // Array used to store the left and right halves
    vector<double> L(n1), R(n2);

    // We copy the data to the temporary arrays for later merging
    for (int i = 0; i < n1; i++)
        L[i] = arr[left + i];
    for (int j = 0; j < n2; j++)
        R[j] = arr[mid + 1 + j];

    // Then merge the temporary arrays back into arr[left..right]
    int i = 0, j = 0, k = left;
    while (i < n1 && j < n2) {
        if (L[i] >= R[j]) {
            arr[k] = L[i];
            i++;
        }
        else {
            arr[k] = R[j];
            j++;
        }
        k++;
    }

    // If there is any remaining element in L[], copy it to arr
    while (i < n1) {
        arr[k] = L[i];
        i++;
        k++;
    }

    // Copy the remaining elements of R[], if there are any
    while (j < n2) {
        arr[k] = R[j];
        j++;
        k++;
    }
}

// Recursive function to sort an array of doubles
void mergeSort(vector<double>& arr, int left, int right) {
    if (left < right) {
        // Same as (left+right)/2, but avoids overflow for large left and right
        int mid = left + (right - left) / 2;

        // Recursively sort both halves
        mergeSort(arr, left, mid);
        mergeSort(arr, mid + 1, right);

        // Merge the sorted halves
        merge(arr, left, mid, right);
    }
}

int main() {
    
    // First, decide how many elements the array will have
    int N;
    cin >> N;

    vector<double> arr(N);
    for (int i = 0; i < N; i++) {
        cin >> arr[i];
    }

    // MergeSort function call
    mergeSort(arr, 0, N - 1);

    // Print the sorted array in descending order, as requested by the activity
    for (int i = 0; i < N; i++) {
        cout << arr[i] << endl;
    }

    return 0;
}
