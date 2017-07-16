# Implementing Naive Bayes

In this exercise you will implement a Gaussian Naive Bayes classifier to predict the behavior of vehicles on a highway. In the image below you can see the behaviors you'll be looking for on a 3 lane highway (with lanes of 4 meter width). The dots represent the d (y axis) and s (x axis) coordinates of vehicles as they either...

1. change lanes left (shown in blue)
2. keep lane (shown in black)
3. or change lanes right (shown in red)
Your job is to write a classifier that can predict which of these three maneuvers a vehicle is engaged in given a single coordinate (sampled from the trajectories shown below).

![Vehicle Behaviours](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/596940c9_naive-bayes/naive-bayes.png)


- Each coordinate contains 4 pieces of information:

1. s
2. d
3. s'
4. d'
​
​-You also know the lane width is 4 meters (this might be helpful in engineering features for your algorithm).

## Instructions
- Implement the `train(self, data, labels)` and `predict(self, observation)` methods in the class GNB in classifier.py


NOTE: You are welcome to use some existing implementation of a Gaussian Naive Bayes classifier. But to get the best results you will still need to put some thought into what features you provide the algorithm when classifying. Though you will only be given the 4 coordinates listed above, you may find that by "engineering" features you may get better performance. For example: the raw value of the d coordinate may not be that useful. But d % lane_width might be helpful since it gives the relative position of a vehicle in it's lane regardless of which lane the vehicle is in.

## Helpful Resources
[sklearn documentation on GaussianNB](http://scikit-learn.org/stable/modules/naive_bayes.html#gaussian-naive-bayes)
[wikipedia article on Naive Bayes / GNB](https://en.wikipedia.org/wiki/Naive_Bayes_classifier#Gaussian_naive_Bayes)
