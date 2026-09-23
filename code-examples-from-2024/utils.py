import umath
def round(value):
    if (abs(value) - umath.floor(abs(value)) < 0.5):
        return umath.floor(value)
    else:
        return umath.ceil(value)
        
