LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY heartbeat IS
    GENERIC (
        G_CLK_FREQ_HZ : INTEGER := 20_000_000 -- 20 MHz
    );
    PORT (
        i_clk        : IN  STD_LOGIC;
        i_rst        : IN  STD_LOGIC;
        o_led        : OUT STD_LOGIC
    );
END heartbeat;

ARCHITECTURE rtl OF heartbeat IS

    -- We need to count to (20,000,000 / 2) - 1 to get a 0.5s toggle (1Hz blink)
    CONSTANT C_COUNTER_LIMIT : INTEGER := (G_CLK_FREQ_HZ / 2) - 1;

    SIGNAL r_counter   : INTEGER RANGE 0 TO C_COUNTER_LIMIT := 0;
    SIGNAL r_led_state : STD_LOGIC := '0';

BEGIN

    PROCESS (i_clk)
    BEGIN
        IF rising_edge(i_clk) THEN
            IF i_rst = '1' THEN
                r_counter   <= 0;
                r_led_state <= '0';
            ELSE
                IF r_counter = C_COUNTER_LIMIT THEN
                    r_counter   <= 0;
                    r_led_state <= NOT r_led_state; -- Toggle the LED
                ELSE
                    r_counter <= r_counter + 1;
                END IF;
            END IF;
        END IF;
    END PROCESS;

    -- Assign the internal state to the output pin
    o_led <= r_led_state;

END rtl;