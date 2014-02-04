""" Final position measurements """
x = [-3.2, -5.2, -0.4, 1.8, -1.1, 0.2, -7.2, 0.0, -1.6, -6.7] #cm
y = [3.2, -0.2, 0.2, 0.4, -2.0, 1.0, -4.3, -3.9, 0.0, -7.1]   #cm

def mean(lst):
    return sum(lst) / len(lst)

def var(lst):
    sum_lst = 0
    avg = mean(lst)
    for item in lst:
        sum_lst += (item - avg)**2
    return sum_lst/len(lst)

def covariance(x, y):
    sum_lst = 0
    x_mean = mean(x)
    y_mean = mean(y)
    for item_x, item_y in zip(x, y):
        sum_lst += (item_x - x_mean) * (item_y - y_mean)
    return sum_lst/len(x)


x_mean = mean(x)
y_mean = mean(y)

x_var = var(x)
y_var = var(y)

cov = covariance(x, y)

print('Mean x and y:')
print('{:.3f}, {:.3f}'.format(x_mean, y_mean))
print()

print('Covariance matrix:')
print('{:.3f} {:.3f}'.format(x_var, cov))
print('{:.3f} {:.3f}'.format(cov, y_var))



