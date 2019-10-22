---
permalink: /how_to_use/
title: "How to use"

toc: true
toc_label: "Menu"
toc_icon: "cog"
---


## How to use

In the following sections you will learn how to use the application.

### Structure of files

Firstly, the sequences must have a specific structure to be read by SLAM-Testbed. The file must be a txt file that should contain all of these fields for each entry:

- Timestamp.
- Position: x, y, z.
- Orientation representated as quaternion: qx, qy, qz, qw.

separating each data by a space and break line after the 'qw'. 

### Load a sequence

Open the application, click on '*File*' at the top menu and then click on '*Open dataset A*' and search your sequence file.

If you only have one sequence, read the next section.

In the case of having two sequences, for example, if you have a sequence and its Groundtruth, load the second secuence in '*File/Open dataset B*' and move to the last section to know how to estimate the transformation between them.
 
### Modify a sequence

After loading one sequence, you can apply different transformations to it to create a new sequence to test SLAM-Testbed.

In order to do so, click on 'Secuence Manager > Modify sequence' to see the transformation menu and choose what transformation you want to do to the original sequence.

### Estimate transformation

Once you have two loaded sequences, you can estimate the transformation between them in order to compare them and obtain different types of information like MSE (Mean square error).

To do this, click on '*Estimator*' and select whatever you prefer: estimate sequence A to B or B to A. After this, wait a few moments and you will see all the information of the estimation.
