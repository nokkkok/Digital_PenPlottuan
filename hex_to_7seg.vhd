library ieee;
use ieee.std_logic_1164.all;

entity hex_to_7seg is
    port (
        i_hex : in  std_logic_vector(3 downto 0);
        o_seg : out std_logic_vector(6 downto 0) -- (g,f,e,d,c,b,a)
    );
end entity hex_to_7seg;

architecture rtl of hex_to_7seg is
begin
    -- Common Cathode: '1' = segment ON, '0' = segment OFF
    p_hex_to_7seg : process (i_hex)
    begin
        case i_hex is
            when x"0" => o_seg <= "0111111"; -- 0
            when x"1" => o_seg <= "0000110"; -- 1
            when x"2" => o_seg <= "1011011"; -- 2
            when x"3" => o_seg <= "1001111"; -- 3
            when x"4" => o_seg <= "1100110"; -- 4
            when x"5" => o_seg <= "1101101"; -- 5
            when x"6" => o_seg <= "1111101"; -- 6
            when x"7" => o_seg <= "0000111"; -- 7
            when x"8" => o_seg <= "1111111"; -- 8
            when x"9" => o_seg <= "1101111"; -- 9
            when x"A" => o_seg <= "1110111"; -- A
            when x"B" => o_seg <= "1111100"; -- b
            when x"C" => o_seg <= "0111001"; -- C
            when x"D" => o_seg <= "1011110"; -- d
            when x"E" => o_seg <= "1111001"; -- E
            when x"F" => o_seg <= "1110001"; -- F
            when others => o_seg <= "0000000"; -- Off
        end case;
    end process p_hex_to_7seg;

end architecture rtl;
