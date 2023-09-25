/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924
 * @brief  Analyzes if the contents of the files mcode1.txt, mcode2.txt, and mcode3.txt are contained in the files
 *         transmission1.txt and transmission2.txt and display a true or false if the chars sequences are contained or not. If
 *         true, it displays true, followed by exactly one space, followed by the position in the transmissionX.txt file where
 *         the mcodeY.txt code starts.
 * @version 0.1
 * @date 09-09-2023
 */

// #pragma GCC optimize("Ofast", "unroll-loops", "no-stack-protector", "fast-math")
// #pragma GCC target("avx,avx2,fma")
#include <bits/stdc++.h>

using namespace std;

typedef long double ld;
typedef long long lli;
typedef pair<lli, lli> ii;
typedef vector<lli> vi;

#define f first
#define s second
#define endl '\n'
#define pb push_back
#define sz(s) lli(s.size())
#define all(s) begin(s), end(s)
#define deb(x) cout << #x ": " << (x) << endl
#define print(x) cout << (x) << endl
#define fore(i, a, b) for (lli i = (a), BS = (b); i < BS; ++i)
#define _                         \
    ios_base::sync_with_stdio(0); \
    cin.tie(0);                   \
    cout.tie(0);

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;

// ... (Definitions and libraries)

/**
 * @brief Reads the content of a file and returns it as a string.
 *
 * @param fileName The name of the file to read.
 * @return A string containing the content of the file.
 * @throws runtime_error If the file fails to open.
 *
 * @note Complexity: O(n), where n is the number of characters in the file.
 */
string readFile(const string &fileName)
{
    ifstream file(fileName);
    if (!file.is_open())
    {
        throw runtime_error("Error opening file: " + fileName);
    }

    string content((istreambuf_iterator<char>(file)), (istreambuf_iterator<char>()));
    file.close();
    return content;
}

/**
 * @brief Checks if a given string is a palindrome.
 *
 * @param s The string to check.
 * @return True if the string is a palindrome, false otherwise.
 *
 * @note Complexity: O(n), where n is the length of the string.
 */
bool isPalindrome(const string &s)
{
    int left = 0;
    int right = s.length() - 1;
    while (left < right)
    {
        if (s[left] != s[right])
        {
            return false;
        }
        left++;
        right--;
    }
    return true;
}

/**
 * @brief Computes the longest border of a string using the KMP algorithm.
 *
 * @param t The string to compute the longest border of.
 * @return A vector containing the longest border of t for each prefix of t.
 *
 * @note Complexity: O(n), where n is the length of the input string t.
 */
vector<int> kmppre(string &t)
{
    vector<int> r(t.size() + 1);
    r[0] = -1;
    int j = -1;
    for (int i = 0; i < (int)(t.size()); ++i)
    {
        while (j >= 0 && t[i] != t[j])
            j = r[j];
        r[i + 1] = ++j;
    }
    return r;
}

/**
 * @brief Finds the first occurrence of a string t in a string s using the KMP algorithm.
 *
 * @param s The string to search in.
 * @param t The string to search for.
 * @return The index of the first occurrence of t in s, or -1 if t is not found in s.
 *
 * @note Complexity: O(n + m), where n is the length of the string s and m is the length of the string t.
 */
int kmp(string &s, string &t)
{
    int match = -1;
    int j = 0;
    vector<int> b = kmppre(t);
    for (int i = 0; i < (int)(s.size()); ++i)
    {
        while (j >= 0 && s[i] != t[j])
            j = b[j];
        if (++j == (int)t.size())
        {
            match = i - j + 1;
            j = b[j];
        }
    }
    return match;
}

/**
 * @brief Uses Manacher's algorithm to find the longest mirrored code (palindrome) in a string.
 *
 * @param s The input string.
 * @return A pair of integers representing the starting and ending positions of the longest mirrored code in s.
 *
 * @note Complexity: O(n), where n is the length of the input string s.
 */
pair<int, int> findLongestMirroredCode(const string &s)
{
    string t = "^#";
    for (char c : s)
    {
        t += c;
        t += '#';
    }
    t += '$';

    int n = t.length();
    vector<int> P(n); // P[i] stores the palindrome radius at position i
    int C = 0;        // Center of the current palindrome
    int R = 0;        // Right boundary of the current palindrome

    for (int i = 1; i < n - 1; ++i)
    {
        int mirror = 2 * C - i; // Mirror of the current position i

        // Check if the current position is within the current palindrome
        if (i < R)
        {
            P[i] = min(R - i, P[mirror]);
        }

        // Attempt to expand the palindrome centered at i
        while (t[i + P[i] + 1] == t[i - P[i] - 1])
        {
            P[i]++;
        }

        // If the expanded palindrome exceeds the right boundary, update C and R
        if (i + P[i] > R)
        {
            C = i;
            R = i + P[i];
        }
    }

    // Find the maximum palindrome radius and its center
    int maxRadius = 0;
    int centerIndex = 0;
    for (int i = 1; i < n - 1; ++i)
    {
        if (P[i] > maxRadius)
        {
            maxRadius = P[i];
            centerIndex = i;
        }
    }

    // Extract the longest mirrored code from the processed string
    int start = (centerIndex - maxRadius) / 2;
    return {start + 1, start + maxRadius};
}

/**
 * @brief Finds the longest common substring between two strings using dynamic programming.
 *
 * @param s1 The first string.
 * @param s2 The second string.
 * @return A pair of integers representing the starting and ending positions of the longest common substring between s1 and s2.
 *
 * @note Complexity: O(n1 * n2), where n1 is the length of the first string and n2 is the length of the second string.
 */
pair<int, int> findLongestCommonSubstring(const string &s1, const string &s2)
{
    int n1 = s1.length();
    int n2 = s2.length();
    vector<vector<int>> dp(n1 + 1, vector<int>(n2 + 1, 0));
    int maxLength = 0;
    int endIndexS1 = 0;

    for (int i = 1; i <= n1; i++)
    {
        for (int j = 1; j <= n2; j++)
        {
            if (s1[i - 1] == s2[j - 1])
            {
                dp[i][j] = dp[i - 1][j - 1] + 1;
                if (dp[i][j] > maxLength)
                {
                    maxLength = dp[i][j];
                    endIndexS1 = i;
                }
            }
        }
    }

    if (maxLength == 0)
    {
        return {-1, -1}; // No common substring found
    }

    return {endIndexS1 - maxLength + 1, endIndexS1};
}

int main()
{
    // Initialize vectors
    vector<string> transmissions(2);
    vector<string> mcodes(3);

    // Read Transmissions files
    for (int i = 0; i < transmissions.size(); i++)
    {
        string fileName = "./Transmissions/transmission" + to_string(i + 1) + ".txt";
        try
        {
            transmissions[i] = readFile(fileName);
        }
        catch (const runtime_error &e)
        {
            cerr << "Error reading transmission file " << fileName << ": " << e.what() << endl;
            exit(1);
        }
    }

    // Read Mcodes files
    for (int i = 0; i < mcodes.size(); i++)
    {
        string fileName = "./MaliciousCode/mcode" + to_string(i + 1) + ".txt";
        try
        {
            mcodes[i] = readFile(fileName);
        }
        catch (const runtime_error &e)
        {
            cerr << "Error reading malicious code file " << fileName << ": " << e.what() << endl;
            exit(1);
        }
    }

    // Part 1: Find mcodes in transmissions
    for (auto &transmission : transmissions)
    {
        for (auto &mcode : mcodes)
        {
            int match = kmp(transmission, mcode);
            if (match != -1)
            {
                cout << "true " << match + 1 << endl;
            }
            else
            {
                cout << "false" << endl;
            }
        }
    }

    // Part 2: Find the longest mirrored code in Transmissions
    for (int i = 0; i < 2; i++)
    {
        pair<int, int> longestMirroredCode = findLongestMirroredCode(transmissions[i]);
        cout << longestMirroredCode.first << " " << longestMirroredCode.second << endl;
    }

    // Part 3: Find the longest common substring between Transmissions
    pair<int, int> longestCommonSubstring = findLongestCommonSubstring(transmissions[0], transmissions[1]);
    cout << longestCommonSubstring.first << " " << longestCommonSubstring.second << endl;

    return 0;
}

// Template obtained from Leones' team for ICPC

// g++-13 -std=c++20 main.cpp && ./a.out
