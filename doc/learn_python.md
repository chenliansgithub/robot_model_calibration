# python宝典

two-dimentional list iterative minus

```python
a = [[1,2,3],[4,5,6]]
b = [[1,4,7],[2,5,8]]
a_b = []
for i in range(len(a)):
    a_b.append([x-y for x,y in zip(a,b)])
```

markers in python

![img](D:\graduateProject\pythonWorkplace\robot_model_calibration\doc\imgs\learning\watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3Z2dnZzMTM=,size_16,color_FFFFFF,t_70)