# Malicious Code Detector & Analyzer in C++

This program is written in C++ to analyze data transmissions and detect potential malicious code.

## Files
The program analyzes five specific text files:

1. `transmission1.txt`
2. `transmission2.txt`
3. `mcode1.txt`
4. `mcode2.txt`
5. `mcode3.txt`

`transmissionX.txt` files represent data sent from one device to another. 
`mcodeX.txt` files represent malicious codes that might be present in a transmission.

## Features

### 1. Malicious Code Detection
This feature checks if the contents of `mcode1.txt`, `mcode2.txt`, and `mcode3.txt` are found in `transmission1.txt` and `transmission2.txt`.

**Output Example**:

```
true 138
false
true 782  
```

### 2. Palindrome (Mirrored) Code Detection
It is believed that the malicious code is always mirrored. Hence, the program also identifies the longest palindrome in the transmission files. 

**Output Example**:

```
startPosition endPosition (for transmission1 file)
startPosition endPosition (for transmission2 file)
```

### 3. Common Substring Analyzer
To understand the similarity between transmissions, the program finds the longest common substring between both transmission files.

**Output Example**:

```
startPosition endPosition (for longest common substring between stream files)
```

## How to Use

1. Place the five required text files in the same directory as the program.
2. Execute the program. There's no need for user input.
3. Review the outputs for malicious code detection, palindrome detection, and common substring analysis.

## Assumptions

- The contents of the files contain characters ranging from 0 to 9, A to F, and line breaks.
- Malicious code is always "mirrored", meaning it's a palindrome.
- Palindrome detection happens at the character level, not at the bits level.
- The longest palindrome will always be found in the transmission.

## How the Program Solves the Problem

### 1. Malicious Code Detection
- The program sequentially reads the characters from each of the `mcodeX.txt` files.
- It then searches for these sequences within the `transmission1.txt` and `transmission2.txt` files.
- If a sequence from an `mcodeX.txt` file is found within a `transmissionX.txt` file, it marks the result as `true` and captures the starting position. If not, it returns `false`.

### 2. Palindrome (Mirrored) Code Detection
- To detect mirrored codes, the program uses a sliding window approach.
- It starts by selecting a window size equivalent to the size of the file and decreases it iteratively until a palindrome is found or the window size is 1.
- For each window size, it checks the sequence of characters to determine if they form a palindrome.
- When it identifies the first palindrome (which will also be the longest due to the decreasing window size approach), it captures the starting and ending positions.

### 3. Common Substring Analyzer
- The program employs a dynamic programming approach to find the longest common substring between `transmission1.txt` and `transmission2.txt`.
- It constructs a 2D table where the cell [i][j] contains the length of the common substring ending at positions i and j of the two files, respectively.
- By updating this table iteratively, the program identifies the length and position of the longest common substring.

These approaches ensure that the program runs efficiently, making it effective even for large transmission files.

## Note

Make sure the required files exist in the directory and are formatted correctly. Ensure no other files conflict with the predetermined filenames.
