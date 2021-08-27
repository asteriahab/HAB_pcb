import pytest
from pcb import pres_init, add_pres_comp, read_pres_digital, read_pres, clear_pres_cal_data, get_pres_cal_data, calc_pres_vars

# Expected calculation results with mocked calibration data and digital readings with values 1
dT = -255
TEMP = 2000+dT/pow(2,23)
OFF = pow(2,17)+dT/pow(2,6)
SENS = pow(2,16)+dT/pow(2,7)

@pytest.fixture
def mocked_readi2c(mocker):
    return mocker.patch('pcb.read_i2c')

@pytest.fixture
def mocked_writei2c(mocker):
    return mocker.patch('pcb.write_i2c')

@pytest.fixture
def mocked_sleep(mocker):
    return mocker.patch('time.sleep')

@pytest.fixture
def mock_pres_cal_data(mocked_readi2c):
    mocked_readi2c.return_value = 1
    pres_init()

    yield

    clear_pres_cal_data()

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
    def pres_init_fail_setup(self):
        pres_init()

        yield

        clear_pres_cal_data()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_pres_init_success(self, pres_init_success_setup, mocked_readi2c):
        mocked_readi2c.return_value = 1
        expected = [1 for _ in range(6)]

        pres_init()
        result = get_pres_cal_data()

        assert result == expected

    def test_pres_init_read_i2c_fail(self, pres_init_fail_setup, mocked_readi2c):
        mocked_readi2c.return_value = None

        pres_init()
        result = get_pres_cal_data()

        assert result is None

#=================================================================================================
# TEST add_pres_comp()
#=================================================================================================
class TestAddPresComp:
    # Temperature inputs to test
    t0, t1, t2, t3 = [2000, 1999, -1500, -1501]
    # Use to make calculations for test cases easier
    d1, d2, d3, d4 = [1/pow(2,31), 61/pow(2,4), pow(t2-2000,2), pow(t3-2000,2)]
    
    # Calculate expected values for TEMP, OFF, and SENS with given input temperatures and dT = OFF = SENS = 1
    params = "dT, TEMP, OFF, SENS, expected"
    above2000   = (1,t0,1,1,[t0,1,1])
    below2000_1 = (1,t1,1,1,[t1-d1, 1-d2, -1])
    below2000_2 = (1,t2,1,1,[t2-d1,1-d2*d3, 1-2*d3])
    belowN1500  = (1,t3,1,1,[t3-d1, 1-d2*d4-15, 1-2*d4-8])
    
    tests = [above2000, below2000_1, below2000_2, belowN1500]

    @pytest.mark.parametrize(params, tests)
    def test_add_pres_comp_success(self, dT, TEMP, OFF, SENS, expected):
        result = add_pres_comp(dT, TEMP, OFF, SENS)

        assert result == expected

#=================================================================================================
# TEST read_pres_digital()
#=================================================================================================
class TestReadPresDigital:
    def test_read_pres_digital_success(self, mocked_readi2c, mocked_writei2c, mocked_sleep):
        mocked_writei2c.return_value = True
        mocked_readi2c.return_value = 1

        result = read_pres_digital(1)

        assert result is not None

    def test_read_pres_digital_write_i2c_fail(self, mocked_writei2c):
        mocked_writei2c.return_value = False

        result = read_pres_digital(1)

        assert result is None

    def test_read_pres_digital_read_i2c_fail(self, mocked_readi2c, mocked_writei2c, mocked_sleep):
        mocked_writei2c.return_value = True
        mocked_readi2c.return_value = None

        result = read_pres_digital(1)

        assert result is None
#=================================================================================================
# TEST calc_pres_vars()
#=================================================================================================
class TestCalcPresVars:
    def test_calc_pres_vars(self, mock_pres_cal_data):     
        result = calc_pres_vars(1)
        
        assert result == [dT, TEMP, OFF, SENS]
        
#=================================================================================================
# TEST read_pres()
#=================================================================================================
class TestReadPres:
    @pytest.fixture
    def mock_readpd(self, mocker):
        return mocker.patch('pcb.read_pres_digital')
    
    @pytest.fixture
    def mock_pinit(self, mocker):
        return mocker.patch('pcb.pres_init')
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_read_pres_success(self, mocker, mock_pres_cal_data, mock_readpd):
        expected_pres = (SENS/pow(2,21) - OFF)/pow(2,15)/100
        expected_temp = TEMP / 100.0
        mock_readpd.return_value = 1
        mocker.patch('pcb.calc_pres_vars', return_value=[dT, TEMP,OFF,SENS])
        mocker.patch('pcb.add_pres_comp', return_value=[TEMP,OFF,SENS])
        
        result_pres, result_temp = read_pres()
        
        assert result_pres == expected_pres and result_temp == expected_temp


    def test_read_pres_no_cal_data(self):
        clear_pres_cal_data()

        result = read_pres()

        assert result is None

    def test_read_pres_no_digital(self, mock_pres_cal_data, mock_readpd, mock_pinit):
        mock_readpd.return_value = None

        result = read_pres()

        assert result is None
