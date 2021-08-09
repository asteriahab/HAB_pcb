import pytest
from wind_tunnel2 import temp_init, read_temp, clear_adt, get_adt

# Dummy exception for mocking
EXCEPT = Exception('mocked')

# Dummy adt objects containing the properties necessary to run temp_init() and read_temp()
ADT_OBJ_1 = type('temp_obj', (object,), {'high_resolution':False, 'temperature':0})
ADT_OBJ_2 = type('temp_obj', (object,), {'high_resolution':False, 'temperature':0})

# Dummy adt objects missing some properties to induce exceptions
ADT_OBJ_NO_RES = type('temp_obj', (object,), {'temperature':0})
ADT_OBJ_NO_TEMP = type('temp_obj', (object,), {'high_resolution':False})

#=================================================================================================
# TEST temp_init()
#=================================================================================================
class TestTempInit:
    @pytest.fixture
    def new_adt(self, mocker):
        clear_adt()
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_1)

        temp_init()

        yield get_adt()

        clear_adt()

    def test_temp_init_success(self, new_adt):
        assert new_adt is not None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @pytest.fixture
    def replace_adt(self, mocker):
        clear_adt()
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_1)
        temp_init()
        prev_adt = get_adt()

        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_2)
        temp_init()
        curr_adt = get_adt()

        yield [prev_adt, curr_adt]

        clear_adt()

    def test_temp_init_replace_success(self, replace_adt):
        assert replace_adt[0] != replace_adt[1]
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @pytest.fixture
    def busio_exception_adt(self, mocker):
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_1)
        temp_init()

        mocker.patch('busio.I2C', side_effect = EXCEPT)
        temp_init()

        yield get_adt()

        clear_adt()

    def test_temp_init_busio_exception(self, busio_exception_adt):
        assert busio_exception_adt is None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @pytest.fixture
    def adt_exception_adt(self, mocker):
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_1)
        temp_init()

        mocker.patch('adafruit_adt7410.ADT7410', side_effect = EXCEPT)
        temp_init()

        yield get_adt()

        clear_adt()

    def test_temp_init_adt_exception(self, adt_exception_adt):
        assert adt_exception_adt is None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @pytest.fixture
    def bad_adt(self, mocker):
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_1)
        temp_init()

        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_NO_RES)
        temp_init()

        yield get_adt()

        clear_adt()

    def test_temp_init_bad_adt_exception(self, bad_adt):
        assert bad_adt is None

#=================================================================================================
# TEST read_temp()
#=================================================================================================
class TestReadTemp:
    @pytest.fixture
    def good_adt(self, mocker):
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_1)
        temp_init()

        mocker.patch('wind_tunnel2.temp_init()')
        yield read_temp()

        clear_adt()

    def test_read_temp_success(self, good_adt):
        assert good_adt is not None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @pytest.fixture
    def bad_adt(self, mocker):
        mocker.patch('busio.I2C', return_value = True)
        mocker.patch('adafruit_adt7410.ADT7410', return_value = ADT_OBJ_NO_TEMP)
        temp_init()

        mocker.patch('wind_tunnel2.temp_init()')
        yield read_temp()

        clear_adt()

    def test_read_temp_exception(self, bad_adt):
        assert bad_adt is None
