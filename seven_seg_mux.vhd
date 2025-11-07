LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY seven_seg_mux IS
    GENERIC (
        G_CLK_FREQ_HZ     : INTEGER := 20_000_000;  -- 20 MHz
        G_REFRESH_RATE_HZ : INTEGER := 800      -- Rate for all 4 digits
    );
    PORT (
        i_clk       : IN  STD_LOGIC;
        i_rst       : IN  STD_LOGIC;
        i_hex_data  : IN  STD_LOGIC_VECTOR(15 DOWNTO 0); -- 4 digits
        o_seg       : OUT STD_LOGIC_VECTOR(6 DOWNTO 0); -- Cathodes (g,f,e,d,c,b,a)
        o_an        : OUT STD_LOGIC_VECTOR(3 DOWNTO 0)  -- Anodes (active-low)
    );
END seven_seg_mux;

ARCHITECTURE rtl OF seven_seg_mux IS

    -- Define a type for the 7-segment mapping
    TYPE t_hex_to_7seg_map IS ARRAY(0 TO 15) OF STD_LOGIC_VECTOR(6 DOWNTO 0);

    -- 7-Segment mapping (Common Cathode) - YOUR CUSTOM MAPPING
    -- (g, f, e, d, c, b, a)
    CONSTANT C_HEX_TO_7SEG : t_hex_to_7seg_map := (
        0  => "0111111", -- 0
        1  => "0000110", -- 1
        2  => "1011011", -- 2
        3  => "1001111", -- 3
        4  => "1100110", -- 4
        5  => "1101101", -- 5
        6  => "1111101", -- 6
        7  => "0000111", -- 7
        8  => "1111111", -- 8
        9  => "1101111", -- 9
        10 => "1110111", -- A
        11 => "1111100", -- b
        12 => "0111001", -- C
        13 => "1011110", -- d
        14 => "1111001", -- E
        15 => "1110001"  -- F
    );

    -- Calculate counter limit for multiplexing
    CONSTANT C_MUX_LIMIT : INTEGER := G_CLK_FREQ_HZ / G_REFRESH_RATE_HZ;
    
    SIGNAL r_mux_cnt    : INTEGER RANGE 0 TO C_MUX_LIMIT - 1 := 0;
    SIGNAL r_digit_sel  : UNSIGNED(1 DOWNTO 0) := "00";
    SIGNAL w_hex_digit  : STD_LOGIC_VECTOR(3 DOWNTO 0);
    
BEGIN

    -- Multiplexing counter
    PROCESS (i_clk)
    BEGIN
        IF rising_edge(i_clk) THEN
            IF i_rst = '1' THEN
                r_mux_cnt <= 0;
                r_digit_sel <= "00";
            ELSE
                IF r_mux_cnt = (C_MUX_LIMIT - 1) THEN
                    r_mux_cnt <= 0;
                    r_digit_sel <= r_digit_sel + 1;
                ELSE
                    r_mux_cnt <= r_mux_cnt + 1;
                END IF;
            END IF;
        END IF;
    END PROCESS;

    -- Select which 4-bit nibble to display
    PROCESS (r_digit_sel, i_hex_data)
    BEGIN
        CASE r_digit_sel IS
            WHEN "00" => -- Digit 0 (Leftmost)
                w_hex_digit <= i_hex_data(15 DOWNTO 12);
            WHEN "01" => -- Digit 1
                w_hex_digit <= i_hex_data(11 DOWNTO 8);
            WHEN "10" => -- Digit 2
                w_hex_digit <= i_hex_data(7 DOWNTO 4);
            WHEN OTHERS => -- Digit 3 (Rightmost)
                w_hex_digit <= i_hex_data(3 DOWNTO 0);
        END CASE;
    END PROCESS;

    -- Set segments based on the selected digit's data
    o_seg <= C_HEX_TO_7SEG(to_integer(unsigned(w_hex_digit)));

    -- Activate the correct anode (active-low)
    PROCESS (r_digit_sel)
    BEGIN
        CASE r_digit_sel IS
            WHEN "00"    => o_an <= "1110"; -- Activate digit 0
            WHEN "01"    => o_an <= "1101"; -- Activate digit 1
            WHEN "10"    => o_an <= "1011"; -- Activate digit 2
            WHEN OTHERS => o_an <= "0111"; -- Activate digit 3
        END CASE;
    END PROCESS;

END rtl;