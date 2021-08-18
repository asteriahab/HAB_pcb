import pytest
from wind_tunnel2.py import pres_init, add_pres_comp, read_pres_digital, read_pres, clear_pres_cal_data, get_pres_cal_data

# Dummy exception for mocking
EXCEPT = Exception('mocked')

#=================================================================================================
# TEST pres_init()
#=================================================================================================
class TestPresInit:
    @pytest.fixture
    def pres_init_success_setup(self):
        clear_pres_cal_data()

        yield

        clear_pres_cal_data()

    @pytest.fixture
    def pres_init_read_i2c_fail_setup(self):
        pres_init()

        yield

        clear_pres_cal_data()

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_pres_init_success(self, mocker, pres_init_success_setup):
        mocker.patch('int.from_bytes', return_value=1)
        expected = [1 for _ in range(6)]

        pres_init()
        result = get_pres_cal_data()

        assert result == expected

    def test_pres_init_read_i2c_fail(self, mocker, pres_init_fail_setup):
        mocker.patch('int.from_bytes', return_value=None)
        expected = None

        pres_init()
        result = get_pres_cal_data()

        assert result == expected

#=================================================================================================
# TEST add_pres_comp()
#=================================================================================================
class TestAddPresComp:
    params = "dt, TEMP, OFF, SENS, expected"
    above2000 = [(1,2001,2,3,[2001,2,3]),(1,2000,2,3,[2000,2,3])]
    below2000 = [(1,1999,1,1,[1999-1/pow(2,31), 1-61/pow(2,4), -1]), (1,-1500,"""FINISH!!""")]
    belowN1500 = [(1,-1501,1,1,["""FINISH!!!!"""])]

    @pytest.mark.parametrize(params, above2000)
    def test_add_pres_comp_success_above2000(self, dT, TEMP, OFF, SENS, expected):
        result = add_pres_comp(dT, TEMP, OFF, SENS)

        assert result == expected

    @pytest.mark.parameterize(params, below2000)
    def test_add_pres_comp_success_below2000(self, dT, TEMP, OFF, SENS, expected):
        result = add_pres_comp(dT, TEMP, OFF, SENS)

        assert result == expected

    @pytest.mark.parameterize(params, belowN1500)
    def test_add_pres_comp_success_belowN1500(self, dT, TEMP, OFF, SENS, expected):
        result = add_pres_comp(dT, TEMP, OFF, SENS)

        assert result == expected

#=================================================================================================
# TEST read_pres_digital()
#=================================================================================================
class TestReadPresDigital:
    def test_read_pres_digital_success(self, mocker):
        mocker.patch('time.sleep')
        mocker.patch('wind_tunnel2.write_i2c', return_value=True)
        mocker.patch('wind_tunnel2.read_i2c', return_value=1)

        result = read_pres_digital(1)

        assert result is not None

    def test_read_pres_digital_write_i2c_fail(self, mocker):
        mocker.patch('wind_tunnel2.write_i2c', return_value=False)

        result = read_pres_digital(1)

        assert result is None

    def test_read_pres_digital_read_i2c_fail(self, mocker):
        mocker.patch('wind_tunnel2.write_i2c', return_value=True)
        mocker.patch('time.sleep')
        mocker.patch('wind_tunnel2.read_i2c', return_value=None)

        result = read_pres_digital(1)

        assert result is None

#=================================================================================================
# TEST read_pres()
#=================================================================================================
class TestReadPres:
    @pytest.fixture
    def mock_pres_cal_data(self, mocker):
        mocker.patch('wind_tunnel2.read_i2c', return_value=1)
        pres_init()

        yield

        clear_pres_cal_data()

    def test_read_pres_success(self, mocker, mock_pres_cal_data):
        mocker.patch('wind_tunnel2.read_pres_digital', return_value=1)
        mocker.patch('wind_tunnel2.add_pres_comp') # FINISH!!!!
        result = read_pres()


    def test_read_pres_no_cal_data(self, mocker):
        mocker.patch('wind_tunnel2.pres_init')
        clear_pres_cal_data()

        result = read_pres()

        assert result is None

    def test_read_pres_no_digital(self, mocker, mock_pres_cal_data):
        mocker.patch('wind_tunnel2.pres_init')
        mocker.patch('wind_tunnel2.read_pres_digital', return_value=None)

        restul = read_pres()

        assert restlt is None
