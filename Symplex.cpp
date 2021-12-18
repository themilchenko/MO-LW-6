#include "Symplex.h"

Symplex::Symplex(std::ifstream& input, bool flag)
{
    std::vector<double> v1;
    std::vector<std::vector<double>> v2;
    std::vector<double> v3;

    /*считываение условие задачи из файла*/
    while (!input.eof())
    {
        std::string current_str;
        std::getline(input, current_str, '\n');
        current_str.erase(
                remove(current_str.begin(), current_str.end(), ' '), current_str.end());

        if ((current_str.find("[[") != std::string::npos) || (current_str.find("[") != std::string::npos))
        {
            current_str.erase(
                    remove(current_str.begin(), current_str.end(), '['), current_str.end());
            current_str.erase(
                    remove(current_str.begin(), current_str.end(), ']'), current_str.end());

            for (auto it = current_str.begin(); it != current_str.end(); it++)
            {
                if (*it == ' ')
                    break;
                std::string res_str = "";
                while (*it != ',')
                    res_str += *it++;
                v1.push_back(std::stod(res_str));
            }

            v2.push_back(std::vector<double>(0));
            for (double el : v1)
                v2[v2.size() - 1].push_back(el);

            v1.clear();
        }
    }

    std::vector<double> last_str_vec;
    if (!flag)
    {
        /*транспонирование матрицы*/
        std::vector<std::vector<double>> transposition_vec(v2[0].size());
        for (int i = 0; i < transposition_vec.size(); ++i)
            transposition_vec[i].resize(v2.size());

        for (int i = 0; i < v2.size(); ++i)
            for (int j = 0; j < v2[i].size(); ++j)
                transposition_vec[j][i] = v2[i][j];

        /*создание симплекс таблицы*/
        table_.resize(transposition_vec.size());
        for (int i = 0; i < transposition_vec.size(); ++i)
        {
            table_[i].resize(transposition_vec[i].size() + 1);
            table_[i][0] = -1;
            for (int j = 0; j < transposition_vec.size(); ++j)
                table_[i][j + 1] = -transposition_vec[i][j];
        }

        //создание последней строки для таблицы
        last_str_vec.resize(table_[0].size());
        last_str_vec[0] = 0;
        for (int i = 1; i < last_str_vec.size(); ++i)
            last_str_vec[i] = -1;
    }
    else
    {
        table_.resize(v2.size());
        for (int i = 0; i < v2.size(); ++i)
        {
            table_[i].push_back(1);
            for (int j = 0; j < v2[i].size(); ++j)
                table_[i].push_back(v2[i][j]);
        }

        //создание последней строки для таблицы
        last_str_vec.resize(table_[0].size());
        last_str_vec[0] = 0;
        for (int i = 1; i < last_str_vec.size(); ++i)
            last_str_vec[i] = 1;
    }

    table_.push_back(last_str_vec);

    /*заполнение векторов базисными и свободными переменными*/
    for (int i = 0; i < table_[0].size() - 1; ++i)
        free_.push_back('x' + std::to_string(i + 1));

    for (int i = table_[0].size(); i < table_[0].size() + table_.size() - 1; ++i)
        basis_.push_back('x' + std::to_string(i));

//    for (int i = 1; i < table_[0].size(); ++i)
//        goal_func.push_back(table_[table_.size() - 1][i]);
}

void Symplex::print() const
{
    size_t bas_indx = 0;

    std::cout << "\t\t" << 'S' << "\t\t";
    for (auto& i : free_)
        std::cout << i << "\t\t";
    std::cout << std::endl;

    for (auto& i : table_)
    {
        if (bas_indx != basis_.size())
        {
            std::cout << basis_[bas_indx] << "  ";
            ++bas_indx;
        }
        else
            std::cout << "F " << "  ";

        for (double j : i)
        {
            if (j == -0.00)
                std::cout << std::setprecision(2) << std::fixed << std::setw(5) << 0.00 << "\t";
            else
                std::cout << std::setprecision(2) << std::fixed << std::setw(5) << j << "\t";
        }
        std::cout << std::endl;
    }
}

bool Symplex::is_optimal() const
{
    size_t counter = 0;
    for (size_t i = 1; i < table_[table_.size() - 1].size(); ++i)
        if (table_[table_.size() - 1][i] <= 0)
            ++counter;

    return (counter == table_[table_.size() - 1].size() - 1) ? false : true;
}

std::pair<int, int> Symplex::find_column_str()
{
    bool is_negative = 1;
    for (int i = 0; i < table_.size() - 1; ++i)
        if (table_[i][0] < 0)
        {
            is_negative = 0;
            bool flag = 0;
            for (int j = 1; j < table_[i].size(); ++j)
                if (table_[i][j] < 0)
                {
                    flag = 1;
                    permissive_column_ = j;
                    break;
                }
            if (flag)
                break;
        }


    if (is_negative)
    {
        bool flag = 1;
        std::pair<int, double> max_element(1, table_[table_.size() - 1][0]);
        for (size_t i = 1; i < table_[table_.size() - 1].size(); ++i)
            if ((table_[table_.size() - 1][i] > 0) && (table_[table_.size() - 1][i] > max_element.second))
            {
                flag = 0;
                max_element = std::pair<int, double>(i, table_[table_.size() - 1][i]);
            }
        permissive_column_ = max_element.first;
        if (flag)
            permissive_column_ = -1;

        int min_index = find_min();

        if (min_index != -1)
        {
            for (size_t i = 0; i < table_.size() - 1; ++i)
                if ((table_[i][permissive_column_] / table_[i][0] > 0) &&
                    (table_[min_index][permissive_column_] / table_[min_index][0] <
                     table_[i][permissive_column_] / table_[i][0]))
                    min_index = i;
            permissive_str_ = min_index;
        }
        else
            permissive_str_ = -1;
        return std::pair<int, int>(permissive_column_, permissive_str_);
    }
    else
    {
        int min_index = find_min();

        if (min_index != -1)
        {
            for (size_t i = 0; i < table_.size() - 1; ++i)
                if ((table_[i][permissive_column_] / table_[i][0] > 0) &&
                    (table_[min_index][permissive_column_] / table_[min_index][0] >
                     table_[i][permissive_column_] / table_[i][0]))
                    min_index = i;
            permissive_str_ = min_index;
        }
        else
            permissive_str_ = -1;

        return std::pair<int, int>(permissive_column_, permissive_str_);
    }
}

int Symplex::find_str() const
{
    int min_index = find_min();

    if (min_index != -1)
    {
        for (size_t i = 0; i < table_.size() - 1; ++i)
            if ((table_[i][permissive_column_] / table_[i][0] > 0) &&
                    (table_[min_index][permissive_column_] / table_[min_index][0] <
                     table_[i][permissive_column_] / table_[i][0]))
                min_index = i;
    }
    else
        return -1;

    return min_index;
}

int Symplex::find_min() const
{
    int min_index = -1;

    for (size_t i = 0; i < table_.size() - 1; ++i)
        if (table_[i][0] / table_[i][permissive_column_] > 0)
            return i;

    return min_index;
}

bool Symplex::do_step()
{
    permissive_column_ = find_column_str().first;
    if (permissive_column_ == -1)
        return 0;
    permissive_str_ = find_column_str().second;
    if (permissive_str_ == -1)
        return 0;

    /*swap переменных xi и xj*/
    std::string temp = basis_[permissive_str_];
    basis_[permissive_str_] = free_[permissive_column_ - 1];
    free_[permissive_column_ - 1] = temp;

    /*создаем матрицу, в кот. буду переписываться новые значения*/
    std::vector<std::vector<double>> copy_to(table_.size());
    for (auto& i : copy_to)
        i.resize(table_[0].size());

    /*переопределяем элементы таблицы*/
    for (size_t i = 0; i < table_.size(); ++i)
        for (size_t j = 0; j < table_[i].size(); ++j)
        {
            if (i == permissive_str_ && j != permissive_column_)
                copy_to[i][j] = table_[i][j] / table_[permissive_str_][permissive_column_];
            else if (i != permissive_str_ && j == permissive_column_)
                copy_to[i][j] = -table_[i][j] / table_[permissive_str_][permissive_column_];
            else if (i == permissive_str_ && j == permissive_column_)
                copy_to[i][j] = 1 / table_[i][j];
            else
                copy_to[i][j] =
                        table_[i][j] - ((table_[permissive_str_][j] * table_[i][permissive_column_]) /
                                        table_[permissive_str_][permissive_column_]);
        }

    table_ = copy_to;
        return 1;
}

void Symplex::get_solution()
{
    std::cout << "W = " << table_[table_.size() - 1][0] << '\n';
    g = 1 / table_[table_.size() - 1][0];
}

void Symplex::do_examination()
{
    std::vector<std::pair<std::string, double>> solutions(free_.size());

    for (size_t i = 0; i < free_.size(); ++i)
        solutions[i].first = "x" + std::to_string(i + 1);


    for (size_t j = 0; j < solutions.size(); ++j)
    {
        bool flag = 0;
        for (size_t i = 0; i < table_.size() - 1; ++i)
            if (solutions[j].first == basis_[i])
            {
                flag = 1;
                solutions[j].second = table_[i][0];
            }
        if (!flag)
            solutions[j].second = 0;
    }

//    for (size_t j = 0; j < solutions.size(); ++j)
//        for (size_t i = 0; i < table_[0].size() - 1; ++i)
//            if (solutions[j].first == free_[i])
//                solutions[j].second = table_[table_.size() - 1][i + 1];

    for (const auto& i : solutions)
        std::cout << i.first << " = " << i.second << std::endl;

    std::cout << "OPTIMAL STRATEGIES:\n";
    std::cout << "g = " << g << '\n';

    for (size_t i = 0; i < free_.size(); ++i)
        solutions[i].first[0] = 'u';

    for (const auto& i : solutions)
        std::cout << i.first << " = " << i.second * g << std::endl;

}

void Symplex::do_dual_examination()
{
    std::vector<std::pair<std::string, double>> solutions(free_.size());

    for (size_t i = 0; i < free_.size(); ++i)
        solutions[i].first = "x" + std::to_string(i + 1);


    for (size_t j = 0; j < solutions.size(); ++j)
    {
        bool flag = 0;
        for (size_t i = 0; i < table_.size() - 1; ++i)
            if (solutions[j].first == basis_[i])
            {
                flag = 1;
                solutions[j].second = table_[i][0];
            }
        if (!flag)
            solutions[j].second = 0;
    }

    for (const auto& i : solutions)
        std::cout << i.first << " = " << i.second << std::endl;

    std::cout << "OPTIMAL STRATEGIES:\n";
    std::cout << "h = " << -g << '\n';

    for (size_t i = 0; i < free_.size(); ++i)
        solutions[i].first[0] = 'v';

    for (const auto& i : solutions)
        std::cout << i.first << " = " << -i.second * g << std::endl;
}
