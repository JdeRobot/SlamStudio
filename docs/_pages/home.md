---
layout: splash
permalink: /
header:
  overlay_color: "#5e616c"
  overlay_image: /assets/images/cover/test_header_shear_3.png
  actions:
    - label: "<i class='fas fa-download'></i> Install now"
      url: "/installation/"
excerpt: 
  Analize your SLAM sequences

feature_row:
  - image_path: /assets/images/cover/cover_column_1.png
    alt: "How to use"
    title: "How to use"
    excerpt: "Learn to use SLAM-Testbed."
    url: "/how_to_use/"
    btn_class: "btn--primary"
    btn_label: "Learn more"

  - image_path: /assets/images/cover/cover_column_2.png
    alt: "fully responsive"
    title: "Contribute"
    excerpt: "Contribute to improve this project."
    url: "/contribute/"
    btn_class: "btn--primary"
    btn_label: "Learn more"

  - image_path: /assets/images/cover/cover_column_3.png
    alt: "100% free"
    title: "More information"
    excerpt: "Find out more about the requirements and objectives of the project."
    url: "/about/"
    btn_class: "btn--primary"
    btn_label: "Learn more"   

---

{% include feature_row %}


## What is SLAM-Testbed?

SLAM-Testbed is a tool in development used to analyze the trajectories obtained by [**SLAM**](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) algorithms. This application allows to:

* Visualize the trajectory on 3D.
* Generate a new sequence from one that already exists and visualize both or load a second sequence.
* Obtain the following estimations projecting one sequence on the other:
  - Scale (x, y, z).
  - Translation (x, y, z).
  - Rotation (yaw, pitch, roll).
  - Time offset.
  - Correlation max value.
  - Root mean square error (RMSE).



