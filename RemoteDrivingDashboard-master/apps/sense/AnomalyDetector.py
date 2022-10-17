import json

import pandas as pd
from sklearn.model_selection import train_test_split
import numpy as np

results = []


def detect_outliers(data):
    results.clear()
    for graph in data:
        series = pd.Series(graph)
        graphDict = series.to_dict()

        # train, test = train_test_split(graph, test_size=0.33) #use if we change to One-class-classification model
        q25, q75 = np.percentile(graph, 25), np.percentile(graph, 75)
        iqr = q75 - q25

        # calculate the outlier cutoff
        cut_off = iqr * 1.5
        lower, upper = q25 - cut_off, q75 + cut_off

        # identify outliers
        outliers = [x for x in graph if x < lower or x > upper]
        resultDict = {}
        for outlier in outliers:
            idx = list(graphDict.keys())[list(graphDict.values()).index(outlier)]

            resultDict[idx] = outlier
            graphDict.pop(idx)

        resultDict = json.dumps(resultDict)
        results.append(resultDict)

    return json.dumps(results)

