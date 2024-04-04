import os
import sys
import pandas as pd


from mplsoccer import PyPizza, add_image, FontManager, Pitch, VerticalPitch, Sbopen
from urllib.request import urlopen

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from Base.SupervisorBase import SupervisorBase
from Utils.Consts import TIME_STEP
from Utils import Functions

class Supervisor(SupervisorBase):
    def __init__(self, file_name: str):
        super().__init__()
        self.file_name = file_name
        self.robot_name = super().RobotList

    def run(self) -> None:
        while super().step(TIME_STEP) != -1:
            super().sendSupervisorData()
            super().isitGoal()
            super().updateScoreboard()
            self.logging()

    def logging(self) -> None:
        data = super().get_data()
        Functions.write_data(self.file_name,data)

    def possession_percentage(self) -> None:
        df = pd.read_csv(self.file_name)
        possession_counts = df.iloc[:, 1].value_counts()
        stats = {name:possession_counts[name] if name in possession_counts.keys() else 0 for name in self.robot_name}
        total = sum(stats.values())

        if total != 0:
            percentages = [round((value / total) * 100, 2) for value in list(stats.values())]
        else:
            percentages = [0] * 8

        Functions.draw_pizza(params=list(stats.keys()), values=percentages, file_name=f'{self.file_name[:-4]}_possession_percentage.png', stat_type='POSSESSION')
    
    def passing_percentage(self) -> None:
        df = pd.read_csv(self.file_name)
        df = df.iloc[:,1].dropna()
        
        stats = {name:0 for name in self.robot_name}
        for i in range(1,len(df)):
            if df.iloc[i-1] != df.iloc[i]:
                if df.iloc[i-1][0] == df.iloc[i][0]:
                    stats[df.iloc[i-1]] += 1
        total = sum(stats.values())

        if total != 0:
            percentages = [round((value / total) * 100, 2) for value in list(stats.values())]
        else:
            percentages = [0] * 8

        Functions.draw_pizza(params=list(stats.keys()), values=percentages, file_name=f'{self.file_name[:-4]}_passing_percentage.png', stat_type='PASSING')

    def interrupt_percentage(self) -> None:
        df = pd.read_csv(self.file_name)
        df = df.iloc[:,1].dropna()
        
        stats = {name:0 for name in self.robot_name}
        for i in range(1,len(df)):
            if df.iloc[i-1] != df.iloc[i]:
                if df.iloc[i-1][0] != df.iloc[i][0]:
                    stats[df.iloc[i]] += 1
        total = sum(stats.values())

        if total != 0:
            percentages = [round((value / total) * 100, 2) for value in list(stats.values())]
        else:
            percentages = [0] * 8

        Functions.draw_pizza(params=list(stats.keys()), values=percentages, file_name=f'{self.file_name[:-4]}_defence_percentage.png', stat_type='DEFENDED')

    def create_heat(self):
        df = pd.read_csv(self.file_name)
        df_pressure_x = df.iloc[:,14]
        df_pressure_y = df.iloc[:,15]

        Functions.draw_heat(df_pressure_x, df_pressure_y, file_name=f'{self.file_name[:-4]}_heatmap.png')
        
        

if __name__ == '__main__':

    file_name = ''
    for i in range(1,10):
        file_name = f'log_game{i}.csv'
        if not os.path.exists(file_name):
            break

    supervisor = Supervisor('log_game1.csv')
    #supervisor.run()
    supervisor.possession_percentage()
    supervisor.passing_percentage()
    supervisor.interrupt_percentage()