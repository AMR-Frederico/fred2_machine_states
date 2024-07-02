from check_if_at_waypoint_helper import SensorsFuzzySystem

fuzzy = SensorsFuzzySystem()
fuzzy.set_rules()
    

def test1():
    result = fuzzy.decide(0, 0, 100, True)
    assert result <  20

def test2():
    result = fuzzy.decide(1, 0, 100, True)
    assert result < 30

def test3():
    result = fuzzy.decide(0, 0, 0, True)
    assert result > 30

def test4():
    result = fuzzy.decide(1, 0, 0, True)
    assert result > 80

def test5():
    result = fuzzy.decide(1, 1, 0, True)
    assert result > 80

def test6():
    result = fuzzy.decide(1, 1, 100, True)
    assert result < 30

def test7():
    result = fuzzy.decide(1, 1, 50, True)
    assert result > 80
