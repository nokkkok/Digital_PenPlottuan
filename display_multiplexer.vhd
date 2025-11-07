library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity display_multiplexer is
    port (
        i_clk         : in  std_logic; -- 20 MHz clock
        i_byte_to_show : in  std_logic_vector(7 downto 0);
        o_seg_anodes   : out std_logic_vector(1 downto 0); -- To control which digit is on (Active-High)
        o_seg_cathodes : out std_logic_vector(6 downto 0)  -- (g,f,e,d,c,b,a) (Active-High)
    );
end entity display_multiplexer;

architecture rtl of display_multiplexer is
    -- Refresh rate: 20,000,000 / 10000 = 2000 Hz. (1kHz per digit)
    constant c_REFRESH_LIMIT : integer := 10000;
    
    signal r_refresh_counter : integer range 0 to c_REFRESH_LIMIT := 0;
    signal r_digit_select : std_logic := '0';
    signal r_nibble : std_logic_vector(3 downto 0);

    -- Component for the hex->7seg converter
    component hex_to_7seg
        port (
            i_hex : in  std_logic_vector(3 downto 0);
            o_seg : out std_logic_vector(6 downto 0)
        );
    end component;

begin

    -- Instantiate the converter
    u_hex_converter : hex_to_7seg
        port map (
            i_hex => r_nibble,
            o_seg => o_seg_cathodes
        );

    -- This process handles the multiplexing (scanning)
    p_multiplexer : process (i_clk)
    begin
        if (rising_edge(i_clk)) then
            if (r_refresh_counter = (c_REFRESH_LIMIT - 1)) then
                r_refresh_counter <= 0;
                r_digit_select <= not r_digit_select; -- Flip to the other digit
            else
                r_refresh_counter <= r_refresh_counter + 1;
            end if;

            if (r_digit_select = '0') then
                -- Show HIGH nibble on Digit 0 (COMMON0)
                r_nibble <= i_byte_to_show(7 downto 4);
                o_seg_anodes <= "01"; -- Enable Digit 0 (active-high)
            else
                -- Show LOW nibble on Digit 1 (COMMON1)
                r_nibble <= i_byte_to_show(3 downto 0);
                o_seg_anodes <= "10"; -- Enable Digit 1 (active-high)
            end if;
        end if;
    end process p_multiplexer;

end architecture rtl;
